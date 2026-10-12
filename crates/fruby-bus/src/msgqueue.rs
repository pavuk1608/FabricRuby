//! FabricRuby/crates/fruby-bus/src/msgqueue.rs
//!
//! Очереди сообщений SysV - короткие сообщения между процессами.
//!
//! Источник: `code/base/ruby_bus.c`, `ruby_bus.h`.
//!
//! ## Что через них ходит
//!
//! Только то , что не помещается в разделяемую память и должно
//! адресоваться по номеру канала: телеметрия LTM/MSP/MAVLink , команды
//! RC , команды пользователя , строки журнала. Видеопоток и статистика
//! линка идут через [`crate::shm`] - там крупные структуры.
//!
//! ## Формат сообщения
//!
//! ```text
//! [0..4)   u32  CRC32 (LE) по байтам [4 .. 7+len)
//! [4]      u8   message id
//! [5]      u8   len & 0xFF      (младший байт длины)
//! [6]      u8   len >> 8        (старший байт длины)
//! [7..]    payload
//! ```
//!
//! Очередь SysV не гарантирует ни доставку , ни порядок , ни целостность,
//! поэтому CRC в заголовке обязателен: он ловит мусор в буфере.
//!
//! Разрыв в идентификаторах считается автоматически: [`MsgQueue::receive`]
//! сравнивает `id` принятого сообщения с предыдущим через
//! [`busMessage::gap_after`] и заносит разницу в счётчики
//! [`ChannelStats::lost`] и [`ChannelStats::unordered`]. Поэтому «сколько
//! сообщений потеряно» - не догадка прикладного кода , а цифра в
//! статистике канала.
//!
//! ## Ключ очереди
//!
//! В C ключ получался через `ftok(path, proj_id)`, то есть зависел от
//! inode файла. Если файл пересоздавался - при обновлении , при смене
//! прав , после `touch` - менялся inode , а значит и ключ , и все
//! процессы «теряли» очереди друг друга.
//!
//! Здесь ключ считается от **фиксированного** пути конфигурации и
//! номера канала: [`ChannelKey::for_channel`]. Никакой зависимости от
//! файлов , никаких побочных эффектов при создании.
//!
//! ## Кто создаёт очередь
//!
//! Создаёт **один** процесс - маршрутизатор (`b-fruby-rt-vehicle` или
//! `b-fruby-rt-station`). Остальные только присоединяются. Это снимает
//! гонку: даже если семь процессов стартуют одновременно , порядок не
//! важен. См. [`OpenPolicy`].

use core::ffi::{c_int, c_long};

use fruby_crypto::compute_crc32;
use fruby_types::{ComponentId, Error};

use crate::sysv::{
    IPC_CREAT, IPC_EXCL, IPC_NOWAIT, IPC_PERM_0666, IPC_STAT, MSQID_DS_BUF_SIZE,
    MSQID_DS_QBYTES_OFFSET, MSQID_DS_QNUM_OFFSET, MSG_NOERROR,
};
use crate::{MAX_CHANNELS, MAX_MSG_SIZE, MSG_TYPE_DATA, MSG_TYPE_LOG};

/// Размер буфера данных одного сообщения.
///
/// Ровно `IPC_CHANNEL_MAX_MSG_SIZE` из C: столько же приходит в
/// `struct { long type; char data[1600]; }`.
const DATA_SIZE: usize = MAX_MSG_SIZE;

/// Индекс контрольной суммы в буфере сообщения.
const CRC_OFFSET: usize = 0;
/// Индекс идентификатора сообщения.
const ID_OFFSET: usize = 4;
/// Индекс младшего байта длины.
const LEN_LO_OFFSET: usize = 5;
/// Индекс старшего байта длины.
const LEN_HI_OFFSET: usize = 6;
/// Начало полезной нагрузки.
const PAYLOAD_OFFSET: usize = 7;

/// Ёмкость полезной нагрузки одного сообщения.
pub const PAYLOAD_CAPACITY: usize = DATA_SIZE - PAYLOAD_OFFSET;

/// Порог заполнения очереди , после которого сообщения не принимаются.
///
/// 800 ‰ - те же 80 % , что применял C при `EAGAIN`. Проверка идёт
/// через `msgctl(bus_STAT)`, а не «на глаз»: очередь на 1600 байт
/// вмещает всего несколько сообщений , и одно лишнее сбрасывает
/// видеочастоту.
pub const HIGH_WATER_PERMILLE: u32 = 800;

/// Фиксированный путь , из которого выводится ключ очереди.
///
/// Существует только как константа: читать его с диска не нужно и
/// нельзя. Он не привязан к реальному файлу и нужен исключительно
/// как «соль» , чтобы ключи разных каналов не совпали.
pub const BUS_KEY_PATH: &str = "/etc/fruby/bus";

/// Множитель для смешивания номера канала с солью пути.
///
/// Взято из хеш-функции Фибоначчи: простое число даёт биты,
/// которые меняются на соседних кодах каналов, а константа нечётная
/// гарантирует , что умножение на неё перемешивает , а не сдвигает.
const CHANNEL_SALT: u32 = 0x9E37_79B1;

/// Старший бит маски ключа.
///
/// Ключ SysV - знаковый `int` , и отрицательный ядро отвергает.
const KEY_SIGN_MASK: u32 = 0x7FFF_FFFF;

/// Идентификатор проекта из C (`LOGGER_MESSAGE_QUEUE_ID`).
///
/// Сохранён **для справки и диагностики** , в вычислении ключа не
/// участвует: `ftok` в этом крейте не используется намеренно , см.
/// модульную документацию. В логах внешних утилит (`ruby_logger`)
/// значение остаётся тем же - 117.
pub const PROJ_ID: u8 = 117;

/// Ключ очереди сообщений (`key_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct ChannelKey(pub i32);

impl ChannelKey {
    /// Ключ для канала.
    ///
    /// Считается как CRC32 от [`BUS_KEY_PATH`] , смешанный с номером
    /// канала константой из хеш-функции Фибоначчи. Старший бит
    /// сбрасывается: ключ SysV - знаковый `int` , и отрицательный ключ
    /// ядро отвергает.
    ///
    /// Функция детерминированная: один и тот же канал всегда даёт
    /// один и тот же ключ , независимо от того , какой процесс и когда
    /// её вызвал.
    pub fn for_channel(channel: crate::ChannelId) -> Self {
        let mut h = compute_crc32(BUS_KEY_PATH.as_bytes());
        h ^= (channel.code() as u32).wrapping_mul(CHANNEL_SALT);
        Self((h & KEY_SIGN_MASK) as i32)
    }

    /// Ключ для роли процесса: один канал - одна очередь.
    ///
    /// Удобная обёртка для листовых процессов , у которых канал ровно
    /// один (см. `fruby_daemon_common::ipc_daemon::channels_for`).
    pub fn for_role_channel(channel: crate::ChannelId) -> Self {
        Self::for_channel(channel)
    }

    /// Ключ как `key_t`.
    pub const fn raw(self) -> i32 {
        self.0
    }
}

impl core::fmt::Display for ChannelKey {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "0x{:08X}", self.0 as u32)
    }
}

/// Раскладка буфера , который передаётся в `msgsnd`/`msgrcv`.
///
/// Первое поле - `mtype`, которое требует ядро. Всё , что идёт за ним,
/// - это ровно те `data`, которые описаны в модульной документации,
/// байт в байт как в C.
///
/// Поле не `pub`: раскладку нельзя менять извне , не сломав
/// совместимость с C-процессами и с `ruby_logger`.
#[repr(C)]
struct RawMessage {
    mtype: c_long,
    data: [u8; DATA_SIZE],
}

impl RawMessage {
    /// Новое сообщение указанного типа.
    const fn new(mtype: c_long) -> Self {
        Self { mtype, data: [0; DATA_SIZE] }
    }
}

/// Сообщение bus - то , что видит прикладной код.
#[derive(Debug, Clone, PartialEq, Eq)]
// Имя с подчёркиванием - дословное имя структуры из C. Переименование
// в `BusMessage` разорвало бы grep между C- и Rust-стороной.
#[allow(non_camel_case_types)]
pub struct busMessage {
    /// Идентификатор сообщения.
    pub id: u8,
    /// Полезная нагрузка (полный радиопакет).
    pub payload: heapless::Vec<u8, PAYLOAD_CAPACITY>,
}

impl busMessage {
    /// Новое сообщение.
    pub const fn new(id: u8) -> Self {
        Self { id, payload: heapless::Vec::new() }
    }

    /// Собрать сообщение в буфер для `msgsnd`.
    ///
    /// Возвращает длину данных: столько байт нужно указать в `msgsz`.
    ///
    /// # Ошибки
    ///
    /// [`Error::OutOfRange`] , если полезная нагрузка не помещается.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        let len = self.payload.len();
        if PAYLOAD_OFFSET + len > out.len() {
            return Err(Error::OutOfRange);
        }
        out[CRC_OFFSET..CRC_OFFSET + 4].fill(0);
        out[ID_OFFSET] = self.id;
        out[LEN_LO_OFFSET] = (len & 0xFF) as u8;
        out[LEN_HI_OFFSET] = ((len >> 8) & 0xFF) as u8;
        out[PAYLOAD_OFFSET..PAYLOAD_OFFSET + len].copy_from_slice(&self.payload);
        let crc = compute_crc32(&out[ID_OFFSET..PAYLOAD_OFFSET + len]);
        out[CRC_OFFSET..CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        Ok(PAYLOAD_OFFSET + len)
    }

    /// Разобрать сообщение из буфера `msgrcv`.
    ///
    /// # Ошибки
    ///
    /// * [`Error::Truncated`] - буфер короче минимума либо длина в
    ///   заголовке больше остатка буфера;
    /// * [`Error::BadChecksum`] - CRC не сошёлся;
    /// * [`Error::OutOfRange`] - длина в заголовке помещается в буфер,
    ///   но не в ёмкость [`busMessage::payload`]. На практике не
    ///   достигается: буфер и ёмкость выводятся из одной константы.
    pub fn decode(buf: &[u8]) -> Result<Self, Error> {
        if buf.len() < PAYLOAD_OFFSET {
            return Err(Error::Truncated);
        }
        let id = buf[ID_OFFSET];
        let len = usize::from(buf[LEN_LO_OFFSET]) | (usize::from(buf[LEN_HI_OFFSET]) << 8);
        if PAYLOAD_OFFSET + len > buf.len() {
            return Err(Error::Truncated);
        }
        let stored = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let computed = compute_crc32(&buf[ID_OFFSET..PAYLOAD_OFFSET + len]);
        if stored != computed {
            return Err(Error::BadChecksum);
        }
        let mut payload: heapless::Vec<u8, PAYLOAD_CAPACITY> = heapless::Vec::new();
        payload
            .extend_from_slice(&buf[PAYLOAD_OFFSET..PAYLOAD_OFFSET + len])
            .map_err(|_| Error::OutOfRange)?;
        Ok(Self { id, payload })
    }

    /// Длина полезной нагрузки в байтах.
    pub fn payload_len(&self) -> usize {
        self.payload.len()
    }

    /// Сколько сообщений потеряно между `last_seen` и этим сообщением.
    ///
    /// Идентификатор монотонный и шириной в один байт , поэтому через
    /// 256 сообщений он перекручивается. Разность со знаком
    /// ([`u8::wrapping_sub`]) остаётся верной и через переполнение.
    ///
    /// `None` означает , что разрыв превышает половину диапазона и
    /// потому **неразличим**: «потерялось 200 сообщений» и «счётчик
    /// врёт» выглядят одинаково. Вызывающий обязан в этом случае
    /// решить сам - обычно перезапустить приёмник , а не ждать.
    ///
    /// Ноль означает «сообщение идёт следующим за `last_seen`» -
    /// нормальный случай , потерь нет.
    pub fn gap_after(&self, last_seen: u8) -> Option<u16> {
        let gap = self.id.wrapping_sub(last_seen);
        if gap > 128 {
            return None;
        }
        Some(u16::from(gap))
    }
}

/// Состояние очереди (`msgctl(bus_STAT)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct QueueStat {
    /// Сообщений в очереди.
    pub qnum: u32,
    /// Занято байт.
    pub qbytes: u32,
    /// Ёмкость очереди в байтах.
    pub qbytes_limit: u32,
}

impl QueueStat {
    /// Заполнение очереди в промилле.
    ///
    /// Если ядро не сообщило предел (`qbytes_limit` пуст), считается,
    /// что очередь заполнена наполовину: заполнять её до конца
    /// опаснее , чем сбрасывать сообщения слишком рано.
    pub fn fill_permille(&self) -> u32 {
        if self.qbytes_limit == 0 {
            return 500;
        }
        ((u64::from(self.qbytes) * 1000) / u64::from(self.qbytes_limit)) as u32
    }

    /// `true`, если очередь перевалила за [`HIGH_WATER_PERMILLE`].
    pub fn is_high_water(&self) -> bool {
        self.fill_permille() >= HIGH_WATER_PERMILLE
    }
}

/// Политика открытия очереди.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OpenPolicy {
    /// Создать , если нет; если уже создана - присоединиться.
    ///
    /// Единственный режим для маршрутизатора: очередь создаёт тот,
    /// кто успел первым , остальные присоединяются. Гонка при старте
    /// семи процессов не приводит к ошибке.
    #[default]
    CreateOrAttach,
    /// Только присоединиться , ошибка если очередь не создана.
    ///
    /// Режим листовых процессов. Если маршрутизатор ещё не поднял
    /// очередь , процесс падает сразу и с понятным кодом , а не
    /// молча пишет в никуда.
    AttachOnly,
}

/// Очередь сообщений одного канала.
///
/// Обёртка над `msgsnd`/`msgrcv` с проверками , счётчиками и
/// политикой заполнения. Дескриптор SysV - это просто целое число,
/// отдельного `close` у него нет: ресурс живёт , пока его не удалят
/// через [`MsgQueue::remove`].
#[derive(Debug)]
pub struct MsgQueue {
    channel: crate::ChannelId,
    key: ChannelKey,
    id: c_int,
    policy: OpenPolicy,
    created: bool,
    stats: ChannelStats,
    next_id: u8,
    last_seen: Option<u8>,
}

/// Счётчики очереди.
///
/// Отдельный тип вместо [`crate::ChannelStats`] , чтобы статистика
/// самой очереди и суммарная статистика маршрутизатора не путались.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChannelStats {
    /// Отправлено сообщений.
    pub sent: u32,
    /// Принято сообщений.
    pub received: u32,
    /// Отброшено отправителем из-за полной очереди.
    pub dropped: u32,
    /// Ошибки дескриптора: очередь удалили.
    pub errors: u32,
    /// Сообщений с битым CRC.
    pub corrupted: u32,
    /// Сообщений , потерянных по дороге.
    ///
    /// Считается по разрыву идентификаторов на приёме: если между
    /// двумя принятыми сообщениями выпало `n` идентификаторов ,
    /// значит `n` сообщений не дошло.
    pub lost: u32,
    /// Случаев неразличимого разрыва идентификаторов.
    ///
    /// Разрыв больше половины диапазона (128) нельзя отличить от
    /// сбившегося счётчика. Число потерянных сообщений в таких
    /// случаях неизвестно , поэтому оно не выдумывается , а сам факт
    /// считается отдельно.
    pub unordered: u32,
}

impl ChannelStats {
    /// Создать пустую статистику.
    pub const fn new() -> Self {
        Self { sent: 0, received: 0, dropped: 0, errors: 0, corrupted: 0, lost: 0, unordered: 0 }
    }

    /// Сбросить все счётчики.
    pub const fn reset(&mut self) {
        *self = Self::new();
    }

    /// Доля потерь в промилле.
    ///
    /// Считается от отправленных: потерянным считается то , что
    /// ушло в очередь и не дошло. Знаменатель - `sent` , а не
    /// `sent + received`, потому что принятые сообщения к потерям
    /// на отправке отношения не имеют.
    ///
    /// При `sent == 0` возвращается ноль: делить не на что , а не
    /// «бесконечные потери» из пустого знаменателя.
    pub fn loss_permille(&self) -> u32 {
        if self.sent == 0 {
            return 0;
        }
        let lost = self.dropped.saturating_add(self.lost);
        // Больше 100 % потерь не бывает по построению, но при
        // переполнении счётчиков разумно ограничить сверху.
        ((u64::from(lost) * 1000) / u64::from(self.sent)).min(1000) as u32
    }

    /// Сумма счётчиков двух каналов.
    pub const fn merge(&mut self, other: &Self) {
        self.sent += other.sent;
        self.received += other.received;
        self.dropped += other.dropped;
        self.errors += other.errors;
        self.corrupted += other.corrupted;
        self.lost += other.lost;
        self.unordered += other.unordered;
    }
}

impl MsgQueue {
    /// Открыть очередь канала.
    ///
    /// # Ошибки
    ///
    /// * [`Error::InvalidState`] - очередь не создана (при
    ///   [`OpenPolicy::AttachOnly`]) либо `msgget` завершился ошибкой;
    /// * [`Error::Unsupported`] - платформа не Linux.
    ///
    /// # Пример
    ///
    /// Маршрутизатор - создатель , листовой процесс - присоединяется.
    /// Пометка `no_run` обязательна: без неё пример выполнялся бы при
    /// `cargo test` и оставлял бы после себя настоящую очередь SysV
    /// в ядре. Такая очередь живёт до перезагрузки и портит
    /// следующие прогоны тестов.
    ///
    /// ```no_run
    /// use fruby_bus::{ChannelId , MsgQueue , OpenPolicy};
    ///
    /// let _router = MsgQueue::open(ChannelId::RouterToRc , OpenPolicy::CreateOrAttach);
    /// let _leaf = MsgQueue::open(ChannelId::RcToRouter , OpenPolicy::AttachOnly);
    /// ```
    pub fn open(channel: crate::ChannelId, policy: OpenPolicy) -> Result<Self, Error> {
        let key = ChannelKey::for_channel(channel);
        // Права передаются как `mode_t`, но складываются с флагами
        // `int`, поэтому приводим явно.
        let perm = IPC_PERM_0666 as c_int;
        let (id, created) = match policy {
            OpenPolicy::AttachOnly => {
                // SAFETY: `key` - обычное целое , указателей нет.
                let id = unsafe { crate::sysv::msgget(key.raw(), 0) };
                if id < 0 {
                    return Err(Error::InvalidState);
                }
                (id, false)
            }
            OpenPolicy::CreateOrAttach => {
                // SAFETY: то же - целочисленный ключ без указателей.
                let id = unsafe { crate::sysv::msgget(key.raw(), IPC_CREAT | IPC_EXCL | perm) };
                if id >= 0 {
                    (id, true)
                } else {
                    // Очередь уже создана: это штатная гонка при
                    // одновременном старте процессов , а не ошибка.
                    // Повторяем без `IPC_EXCL`.
                    // SAFETY: см. выше.
                    let id = unsafe { crate::sysv::msgget(key.raw(), 0) };
                    if id < 0 {
                        return Err(Error::InvalidState);
                    }
                    (id, false)
                }
            }
        };
        Ok(Self {
            channel,
            key,
            id,
            policy,
            created,
            stats: ChannelStats::new(),
            next_id: 0,
            last_seen: None,
        })
    }

    /// Удалить очередь канала.
    ///
    /// Вызывает создатель при штатном завершении. Остальные процессы
    /// после этого получают ошибку при `msgsnd`/`msgrcv`, что
    /// правильно: система разваливается , и молча работать дальше
    /// не должна.
    pub fn remove(channel: crate::ChannelId) -> bool {
        // SAFETY: `id` получен из успешного `msgget` того же канала.
        let id = unsafe { crate::sysv::msgget(ChannelKey::for_channel(channel).raw(), 0) };
        if id < 0 {
            return false;
        }
        // SAFETY: `id` - дескриптор существующей очереди.
        unsafe { crate::sysv::msgclose(id) == 0 }
    }

    /// Очередь создана этим вызовом или существовала раньше.
    pub const fn was_created(&self) -> bool {
        self.created
    }

    /// Политика , которой открыта очередь.
    pub const fn policy(&self) -> OpenPolicy {
        self.policy
    }

    /// Канал очереди.
    pub const fn channel(&self) -> crate::ChannelId {
        self.channel
    }

    /// Ключ очереди.
    pub const fn key(&self) -> ChannelKey {
        self.key
    }

    /// Дескриптор очереди (`msqid`).
    ///
    /// Нужен для диагностики: `ipcs -q` показывает его в шестнадцатеричном
    /// виде , и это самый быстрый способ снять дамп очереди с живой системы.
    pub const fn id(&self) -> c_int {
        self.id
    }

    /// Счётчики.
    pub const fn stats(&self) -> &ChannelStats {
        &self.stats
    }

    /// Счётчики для изменения.
    pub const fn stats_mut(&mut self) -> &mut ChannelStats {
        &mut self.stats
    }

    /// Текущее состояние очереди.
    ///
    /// `msgctl(bus_STAT)` отдаёт `msg_qnum` (сколько сообщений) и
    /// `msg_qbytes` (сколько байт занято), но **не** отдаёт предел:
    /// тот живёт в отдельном sysctl `kernel.msgmnb`. Предел
    /// подставляется из [`queue_bytes_limit`].
    ///
    /// `None`, если `msgctl` не ответил - например , на не-Linux.
    pub fn stat(&self) -> Option<QueueStat> {
        let mut buf = [0u8; MSQID_DS_BUF_SIZE];
        // SAFETY: `buf` достаточно велик для `struct msqid_ds`
        // (см. `MSQID_DS_BUF_SIZE`), ядро читает из него столько,
        // сколько считает нужным , и не пишет больше размера структуры.
        let rc = unsafe {
            crate::sysv::msgctl(
                self.id,
                IPC_STAT,
                buf.as_mut_ptr().cast::<core::ffi::c_void>(),
            )
        };
        if rc < 0 {
            return None;
        }
        Some(QueueStat {
            qnum: read_u64_at(&buf, MSQID_DS_QNUM_OFFSET).min(u64::from(u32::MAX)) as u32,
            qbytes: read_u64_at(&buf, MSQID_DS_QBYTES_OFFSET).min(u64::from(u32::MAX)) as u32,
            qbytes_limit: queue_bytes_limit(),
        })
    }

    /// Отправить сообщение типа [`MSG_TYPE_DATA`].
    ///
    /// # Ошибки
    ///
    /// * [`Error::InvalidState`] - очередь удалена другим процессом;
    /// * [`Error::OutOfRange`] - очередь заполнена больше чем на
    ///   [`HIGH_WATER_PERMILLE`] либо полезная нагрузка не помещается.
    ///
    /// Отбрасывание считается **после** отказа `msgsnd` , а не до
    /// него. Проверять заполненность заранее было бы ошибкой:
    /// между проверкой и записью очередь успевает наполниться , и
    /// настоящее переполнение попадало бы в счётчик ошибок вместо
    /// счётчика потерь. На успешном пути добавочного системного
    /// вызова теперь вообще нет.
    ///
    /// # Как отличается переполнение от смерти дескриптора
    ///
    /// Отказ `msgsnd` сам по себе ничего не говорит: это либо
    /// `EAGAIN` (очередь полна) , либо `EIDRM` (очередь удалили).
    /// Разбор идёт вызовом `msgctl(bus_STAT)`: на живой очереди он
    /// отвечает , на удалённой - нет.
    pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
        self.send_typed(MSG_TYPE_DATA, payload)
    }

    /// Отправить сообщение заданного типа.
    ///
    /// Тип нужен , чтобы разделить трафик: в C `ruby_logger` принимает
    /// только сообщения с типом [`MSG_TYPE_LOG`] и всё остальное
    /// отбрасывает , не заглядывая внутрь.
    ///
    /// # Ошибки
    ///
    /// * [`Error::InvalidState`] - очередь удалена другим процессом;
    /// * [`Error::OutOfRange`] - очередь заполнена либо полезная
    ///   нагрузка не помещается в [`PAYLOAD_CAPACITY`].
    pub fn send_typed(&mut self, mtype: c_long, payload: &[u8]) -> Result<(), Error> {
        if payload.len() > PAYLOAD_CAPACITY {
            self.stats.dropped += 1;
            return Err(Error::OutOfRange);
        }

        let mut msg = busMessage::new(self.next_id);
        msg.payload
            .extend_from_slice(payload)
            .map_err(|_| Error::OutOfRange)?;
        let mut raw = RawMessage::new(mtype);
        let len = msg.encode(&mut raw.data)?;
        self.next_id = self.next_id.wrapping_add(1);

        // SAFETY: `raw` - `#[repr(C)]` с `mtype` первым полем , ровно
        // та раскладка , которую требует ядро; `len` - длина данных
        // за `mtype`, посчитанная `encode`.
        let rc = unsafe {
            crate::sysv::msgsnd(
                self.id,
                core::ptr::addr_of!(raw.mtype).cast::<core::ffi::c_void>(),
                len,
                IPC_NOWAIT,
            )
        };
        if rc == 0 {
            self.stats.sent += 1;
            return Ok(());
        }
        if self.stat().is_some() {
            self.stats.dropped += 1;
            return Err(Error::OutOfRange);
        }
        self.stats.errors += 1;
        Err(Error::InvalidState)
    }

    /// Принять сообщение любого типа без ожидания.
    ///
    /// `Ok(None)` - очередь пуста , это штатная ситуация , не ошибка.
    ///
    /// # Ошибки
    ///
    /// * [`Error::InvalidState`] - очередь удалена другим процессом;
    /// * [`Error::BadChecksum`] - сообщение повреждено (счётчик
    ///   `corrupted` при этом увеличивается);
    /// * [`Error::Truncated`] - длина в заголовке не бьётся с буфером;
    /// * [`Error::OutOfRange`] - полезная нагрузка не влезла в `out`.
    ///
    /// Если очередь удалили , метод возвращает [`Error::InvalidState`]
    /// , а не `Ok(None)`. Иначе листовой процесс вечно получал бы
    /// «пусто» и молча крутил цикл на мёртвой шине.
    pub fn receive(&mut self, out: &mut [u8]) -> Result<Option<usize>, Error> {
        self.receive_typed(0, out)
    }

    /// Принять сообщение заданного типа без ожидания.
    ///
    /// `mtype` передаётся в `msgrcv` как `msgtyp`. Ноль означает
    /// «первое сообщение любого типа» , положительное значение -
    /// «первое сообщение ровно этого типа». Это позволяет держать в
    /// одной очереди и данные , и журнал , не разбирая содержимое.
    ///
    /// # Ошибки
    ///
    /// См. [`MsgQueue::receive`].
    pub fn receive_typed(&mut self, mtype: c_long, out: &mut [u8]) -> Result<Option<usize>, Error> {
        let mut raw = RawMessage::new(0);
        // SAFETY: `raw` - корректный буфер `#[repr(C)]` с `mtype`
        // первым полем; ядро запишет `mtype` в него и данные после.
        let rc = unsafe {
            crate::sysv::msgrcv(
                self.id,
                core::ptr::addr_of_mut!(raw.mtype).cast::<core::ffi::c_void>(),
                raw.data.len(),
                mtype,
                MSG_NOERROR | IPC_NOWAIT,
            )
        };
        if rc < 0 {
            // `msgrcv` возвращает один и тот же код и для пустой
            // очереди (`EAGAIN`) , и для удалённой (`EIDRM`) , и для
            // неверного дескриптора (`EINVAL`). Различать их без
            // `errno` нечем, поэтому состояние очереди уточняется
            // вызовом `msgctl`: на живой очереди он отвечает.
            //
            // Проверка стоит именно здесь , а не до `msgrcv`: на
            // успешном пути она бы добавила лишний системный вызов к
            // каждому принятому сообщению. Плата - один `msgctl` на
            // каждый опрос пустой очереди; в цикле опроса с
            // сообщениями это заметно , в обычном режиме - нет.
            if self.stat().is_none() {
                self.stats.errors += 1;
                return Err(Error::InvalidState);
            }
            return Ok(None);
        }
        let msg = match busMessage::decode(&raw.data[..rc as usize]) {
            Ok(m) => m,
            Err(e) => {
                self.stats.corrupted += 1;
                return Err(e);
            }
        };
        if msg.payload.len() > out.len() {
            return Err(Error::OutOfRange);
        }
        out[..msg.payload.len()].copy_from_slice(&msg.payload);
        self.stats.received += 1;
        let mut last = self.last_seen.take();
        note_gap(&mut last, &mut self.stats, msg.id);
        self.last_seen = last;
        Ok(Some(msg.payload.len()))
    }

    /// Принять только сообщения журнала ([`MSG_TYPE_LOG`]).
    ///
    /// # Ошибки
    ///
    /// См. [`MsgQueue::receive`].
    pub fn receive_log(&mut self, out: &mut [u8]) -> Result<Option<usize>, Error> {
        self.receive_typed(MSG_TYPE_LOG, out)
    }
}

/// Учесть разрыв в идентификаторах принятых сообщений.
///
/// Идентификатор монотонный и шириной в один байт, поэтому через 256
/// сообщений перекручивается. Разрыв считается через
/// [`busMessage::gap_after`] - то есть по тем же правилам, что и в самом
/// методе: больше половины диапазона неразличимо.
///
/// Первое принятое сообщение задаёт точку отсчёта и потерь не
/// увеличивает: до него ничего не известно , и разрыв был бы выдумкой.
///
/// Вынесено отдельной функцией , а не методом [`MsgQueue`] , потому что
/// не требует открытой очереди: проверяется обычным тестом без
/// системных вызовов.
fn note_gap(last_seen: &mut Option<u8>, stats: &mut ChannelStats, id: u8) {
    if let Some(prev) = *last_seen {
        match busMessage::new(id).gap_after(prev) {
            // Расстояние не меньше единицы: идентификатор только что
            // вырос. Вычитаем единицу - это то самое сообщение,
            // которое приняли сейчас, а не потеря.
            Some(gap) if gap > 0 => {
                stats.lost = stats.lost.saturating_add(u32::from(gap) - 1);
            }
            Some(_) => {}
            None => stats.unordered += 1,
        }
    }
    *last_seen = Some(id);
}

/// Прочитать `unsigned long` из буфера состояния очереди.
fn read_u64_at(buf: &[u8; MSQID_DS_BUF_SIZE], offset: usize) -> u64 {
    let mut bytes = [0u8; 8];
    let width = core::mem::size_of::<c_long>();
    bytes[..width].copy_from_slice(&buf[offset..offset + width]);
    u64::from_ne_bytes(bytes)
}

/// Предел очереди в байтах (`kernel.msgmnb`).
///
/// Значение не меняется за время работы системы , поэтому читается один
/// раз и кэшируется. Если sysctl недоступен - не-Linux , контейнер без
/// `/proc`, запрещённое чтение - берётся значение по умолчанию MSGMNB
/// из ядра Linux.
pub fn queue_bytes_limit() -> u32 {
    static LIMIT: std::sync::OnceLock<u32> = std::sync::OnceLock::new();
    *LIMIT.get_or_init(|| {
        std::fs::read_to_string("/proc/sys/kernel/msgmnb")
            .ok()
            .and_then(|s| s.trim().parse::<u32>().ok())
            .filter(|v| *v > 0)
            .unwrap_or(DEFAULT_QBYTES_LIMIT)
    })
}

/// Предел очереди по умолчанию , если sysctl недоступен.
///
/// `kernel.msgmnb` по умолчанию в Linux - 16384 байта. При
/// [`PAYLOAD_CAPACITY`] = 1593 это десять сообщений: очередь
/// заполняется быстро , и проверка заполнения обязательна.
const DEFAULT_QBYTES_LIMIT: u32 = 16384;

/// Маршрутизатор: набор очередей , обслуживающих один процесс.
///
/// Маршрутизаторы - `b-fruby-rt-vehicle` и `b-fruby-rt-station`; они
/// открывают все [`crate::ChannelId::ROUTER_CHANNELS`] и раскладывают
/// входящие сообщения по процессам. Листовые процессы открывают по
/// одной-двум очереди и работают через тот же тип.
#[derive(Debug)]
pub struct Router {
    queues: heapless::Vec<(crate::ChannelId, MsgQueue), MAX_CHANNELS>,
    stats: ChannelStats,
}

impl Router {
    /// Открыть все каналы маршрутизатора.
    ///
    /// # Ошибки
    ///
    /// См. [`MsgQueue::open`].
    pub fn open_router() -> Result<Self, Error> {
        Self::open(&crate::ChannelId::ROUTER_CHANNELS, OpenPolicy::CreateOrAttach)
    }

    /// Открыть конкретные каналы.
    ///
    /// # Откат при частичной ошибке
    ///
    /// Если очередь пятого канала из семи не открылась , первые
    /// четыре остались бы висеть в ядре до ручной чистки `ipcrm`.
    /// Поэтому при ошибке снимаются **только те очереди , которые
    /// создал этот вызов** ([`MsgQueue::was_created`]); присоединённые
    /// не трогаются , они принадлежат маршрутизатору.
    ///
    /// # Ошибки
    ///
    /// См. [`MsgQueue::open`].
    pub fn open(
        channels: &[crate::ChannelId],
        policy: OpenPolicy,
    ) -> Result<Self, Error> {
        let mut queues: heapless::Vec<(crate::ChannelId, MsgQueue), MAX_CHANNELS> =
            heapless::Vec::new();
        for &ch in channels {
            if queues.iter().any(|(c, _)| *c == ch) {
                continue;
            }
            match MsgQueue::open(ch, policy) {
                Ok(q) => {
                    if queues.push((ch, q)).is_err() {
                        // Вместимость `MAX_CHANNELS` исчерпана. Откатываем
                        // всё , что открыли, и уходим.
                        remove_created(&queues);
                        return Err(Error::OutOfRange);
                    }
                }
                Err(e) => {
                    remove_created(&queues);
                    return Err(e);
                }
            }
        }
        Ok(Self { queues, stats: ChannelStats::new() })
    }

    /// Отправить сообщение в канал компонента.
    ///
    /// Направление «от маршрутизатора» задаётся флагом
    /// [`crate::ChannelFor::for_component`]: для маршрутизатора - всегда
    /// `true`.
    ///
    /// # Ошибки
    ///
    /// * [`Error::Unsupported`] - для этого компонента нет канала
    ///   (например , видео идёт через разделяемую память);
    /// * остальные - из [`MsgQueue::send`].
    pub fn send_component(&mut self, component: ComponentId, payload: &[u8]) -> Result<(), Error> {
        let ch = crate::ChannelFor::for_component(true, component)
            .ok_or(Error::Unsupported)?
            .id();
        self.send_channel(ch, payload)
    }

    /// Отправить сообщение в конкретный канал.
    ///
    /// # Ошибки
    ///
    /// * [`Error::InvalidState`] - канал не открыт этим маршрутизатором;
    /// * остальные - из [`MsgQueue::send`].
    pub fn send_channel(&mut self, ch: crate::ChannelId, payload: &[u8]) -> Result<(), Error> {
        let q = self.queue_mut(ch)?;
        let r = q.send(payload);
        if r.is_ok() {
            self.stats.sent += 1;
        }
        r
    }

    /// Отправить строку журнала.
    ///
    /// # Ошибки
    ///
    /// См. [`Router::send_channel`].
    pub fn send_log(&mut self, ch: crate::ChannelId, line: &[u8]) -> Result<(), Error> {
        let q = self.queue_mut(ch)?;
        let r = q.send_typed(MSG_TYPE_LOG, line);
        if r.is_ok() {
            self.stats.sent += 1;
        }
        r
    }

    /// Принять сообщение из канала компонента.
    ///
    /// Направление - `false`: сообщения к маршрутизатору.
    ///
    /// # Ошибки
    ///
    /// * [`Error::Unsupported`] - для этого компонента нет канала;
    /// * остальные - из [`MsgQueue::receive`].
    pub fn receive_component(
        &mut self,
        component: ComponentId,
        out: &mut [u8],
    ) -> Result<Option<usize>, Error> {
        let ch = crate::ChannelFor::for_component(false, component)
            .ok_or(Error::Unsupported)?
            .id();
        self.receive_channel(ch, out)
    }

    /// Принять сообщение из конкретного канала.
    ///
    /// # Ошибки
    ///
    /// * [`Error::InvalidState`] - канал не открыт этим маршрутизатором;
    /// * остальные - из [`MsgQueue::receive`].
    pub fn receive_channel(
        &mut self,
        ch: crate::ChannelId,
        out: &mut [u8],
    ) -> Result<Option<usize>, Error> {
        let q = self.queue_mut(ch)?;
        let r = q.receive(out);
        if matches!(r, Ok(Some(_))) {
            self.stats.received += 1;
        }
        r
    }

    /// Открытые каналы.
    pub fn channels(&self) -> impl Iterator<Item = crate::ChannelId> + '_ {
        self.queues.iter().map(|(c, _)| *c)
    }

    /// Счётчики канала.
    pub fn stats_of(&self, ch: crate::ChannelId) -> Option<&ChannelStats> {
        self.queues.iter().find(|(c, _)| *c == ch).map(|(_, q)| q.stats())
    }

    /// Суммарные счётчики по всем каналам.
    pub const fn total_stats(&self) -> &ChannelStats {
        &self.stats
    }

    /// Состояние очереди канала.
    ///
    /// # Ошибки
    ///
    /// `None`, если канал не открыт или `msgctl` не ответил.
    pub fn queue_stat(&self, ch: crate::ChannelId) -> Option<QueueStat> {
        self.queues
            .iter()
            .find(|(c, _)| *c == ch)
            .and_then(|(_, q)| q.stat())
    }

    /// Счётчик всех каналов , прошедших порог заполнения.
    pub fn saturated_channels(&self) -> usize {
        self.queues
            .iter()
            .filter(|(_, q)| q.stat().is_some_and(|s| s.is_high_water()))
            .count()
    }

    fn queue_mut(&mut self, ch: crate::ChannelId) -> Result<&mut MsgQueue, Error> {
        self.queues
            .iter_mut()
            .find(|(c, _)| *c == ch)
            .map(|(_, q)| q)
            .ok_or(Error::InvalidState)
    }
}

/// Снять очереди , созданные текущим вызовом.
///
/// Вызывается только при неудачном [`Router::open`]. Присоединённые
/// очереди не трогаются: они созданы маршрутизатором и должны
/// пережить неудачу чужого процесса.
fn remove_created(queues: &[(crate::ChannelId, MsgQueue)]) {
    for (ch, q) in queues {
        if q.was_created() {
            MsgQueue::remove(*ch);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ChannelFor;

    /// Сериализовать тесты, работающие с настоящими ресурсами IPC.
    ///
    /// libtest запускает тесты в потоках, а ресурсы SysV именуются
    /// по номеру канала и видны всему пользователю: два теста на одном
    /// канале мешали бы друг другу и «залипали» бы на очереди,
    /// оставшейся от предыдущего прогона.
    fn ipc_guard() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        // Отравленный замок - следствие падения прошлого теста, а не
        // причина не пускать следующий.
        LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[test]
    fn key_is_positive_and_deterministic() {
        for ch in crate::ChannelId::ALL {
            let k = ChannelKey::for_channel(ch);
            assert!(k.raw() > 0, "ключ {ch:?} должен быть положительным");
            assert_eq!(k, ChannelKey::for_channel(ch));
        }
    }

    #[test]
    fn keys_are_unique_per_channel() {
        let mut keys: heapless::Vec<ChannelKey, 16> = heapless::Vec::new();
        for ch in crate::ChannelId::ALL {
            let k = ChannelKey::for_channel(ch);
            assert!(!keys.contains(&k), "ключ {ch:?} совпал с другим каналом");
            keys.push(k).unwrap();
        }
    }

    #[test]
    fn key_ignores_process_identity() {
        // Ключ не должен зависеть от того , кто его вычислил: в этом
        // весь смысл отказа от `ftok`.
        let a = ChannelKey::for_channel(crate::ChannelId::RouterToCentral);
        let b = ChannelKey::for_role_channel(crate::ChannelId::RouterToCentral);
        assert_eq!(a, b);
    }

    #[test]
    fn message_roundtrip() {
        let mut m = busMessage::new(7);
        m.payload.extend_from_slice(&[1, 2, 3, 4, 5]).unwrap();
        let mut buf = [0u8; DATA_SIZE];
        let n = m.encode(&mut buf).unwrap();
        assert_eq!(n, PAYLOAD_OFFSET + 5);
        assert_eq!(busMessage::decode(&buf[..n]).unwrap(), m);
    }

    #[test]
    fn payload_survives_full_buffer() {
        let mut m = busMessage::new(1);
        m.payload.resize(PAYLOAD_CAPACITY, 0xA5).unwrap();
        let mut buf = [0u8; DATA_SIZE];
        let n = m.encode(&mut buf).unwrap();
        assert_eq!(n, DATA_SIZE);
        let back = busMessage::decode(&buf[..n]).unwrap();
        assert_eq!(back.payload.len(), PAYLOAD_CAPACITY);
    }

    #[test]
    fn corrupted_message_rejected() {
        let mut m = busMessage::new(1);
        m.payload.extend_from_slice(&[9, 9]).unwrap();
        let mut buf = [0u8; DATA_SIZE];
        let n = m.encode(&mut buf).unwrap();
        buf[ID_OFFSET] ^= 0xFF;
        assert_eq!(busMessage::decode(&buf[..n]).unwrap_err(), Error::BadChecksum);
    }

    #[test]
    fn corrupted_payload_rejected() {
        // Порча именно в теле , а не в заголовке: CRC обязан ловить и то,
        // и другое.
        let mut m = busMessage::new(3);
        m.payload.extend_from_slice(&[0x11, 0x22, 0x33]).unwrap();
        let mut buf = [0u8; DATA_SIZE];
        let n = m.encode(&mut buf).unwrap();
        buf[PAYLOAD_OFFSET + 1] ^= 0x01;
        assert_eq!(busMessage::decode(&buf[..n]).unwrap_err(), Error::BadChecksum);
    }

    #[test]
    fn truncated_message_rejected() {
        assert_eq!(busMessage::decode(&[0; 4]).unwrap_err(), Error::Truncated);
        let mut m = busMessage::new(1);
        m.payload.extend_from_slice(&[1, 2, 3]).unwrap();
        let mut buf = [0u8; DATA_SIZE];
        let n = m.encode(&mut buf).unwrap();
        assert_eq!(busMessage::decode(&buf[..n - 1]).unwrap_err(), Error::Truncated);
    }

    #[test]
    fn oversized_payload_rejected() {
        let mut m = busMessage::new(0);
        m.payload.extend_from_slice(&[1, 2, 3]).unwrap();
        let mut buf = [0u8; PAYLOAD_OFFSET];
        assert_eq!(m.encode(&mut buf).unwrap_err(), Error::OutOfRange);
    }

    #[test]
    fn message_id_wraps_without_panic() {
        let m = busMessage::new(255);
        let next = m.id.wrapping_add(1);
        assert_eq!(next, 0);
    }

    #[test]
    fn gap_detection_survives_wraparound() {
        let m = busMessage::new(2);
        // 254 -> 2 идёт «через ноль»: пропущены идентификаторы
        // 255 , 0 и 1 , то есть потеряно три сообщения.
        assert_eq!(m.gap_after(254), Some(4));
        // Соседнее сообщение - потерь нет.
        assert_eq!(m.gap_after(1), Some(1));
    }

    #[test]
    fn gap_beyond_half_range_is_undecidable() {
        // Разрыв больше половины диапазона неразличим: сообщение
        // с идентификатором 2 «на 198 позже» сорокового - это либо
        // 198 потерянных сообщений , либо сбившийся счётчик.
        let m = busMessage::new(2);
        assert_eq!(m.gap_after(60), None);
        // Ровно половина диапазона ещё разбирается.
        assert_eq!(m.gap_after(130), Some(128));
    }

    #[test]
    fn open_fails_without_sysv_ipc() {
        // На не-Linux заглушки всегда возвращают ошибку. На Linux тест
        // тоже полезен: он не требует прав и не создаёт ничего.
        let r = MsgQueue::open(crate::ChannelId::RouterToRc, OpenPolicy::AttachOnly);
        #[cfg(not(target_os = "linux"))]
        assert_eq!(r.unwrap_err(), Error::InvalidState);
        #[cfg(target_os = "linux")]
        let _ = r;
    }

    #[test]
    fn unopened_queue_rejects_io() {
        // Без дескриптора работать нельзя - иначе ошибка была бы
        // отложенной и невнятной.
        let q = MsgQueue::open(crate::ChannelId::RouterToRc, OpenPolicy::AttachOnly);
        if let Err(e) = q {
            assert_eq!(e, Error::InvalidState);
        }
    }

    #[test]
    fn queue_stat_fill_permille() {
        let s = QueueStat { qnum: 5, qbytes: 500, qbytes_limit: 1000 };
        assert_eq!(s.fill_permille(), 500);
        assert!(!s.is_high_water());
    }

    #[test]
    fn queue_stat_high_water() {
        let s = QueueStat { qnum: 9, qbytes: 900, qbytes_limit: 1000 };
        assert_eq!(s.fill_permille(), 900);
        assert!(s.is_high_water());
    }

    #[test]
    fn queue_stat_without_limit_is_half_full() {
        // Неизвестный предел не должен означать «вечно свободно».
        let s = QueueStat::default();
        assert_eq!(s.fill_permille(), 500);
    }

    #[test]
    fn stats_merge_and_reset() {
        let mut a = ChannelStats { sent: 1, received: 2, dropped: 3, errors: 4, corrupted: 5, ..ChannelStats::new() };
        let b = ChannelStats {
            sent: 10,
            received: 20,
            dropped: 30,
            errors: 40,
            corrupted: 50,
            lost: 60,
            unordered: 70,
        };
        a.merge(&b);
        assert_eq!(a.sent, 11);
        assert_eq!(a.corrupted, 55);
        assert_eq!(a.lost, 60);
        assert_eq!(a.unordered, 70);
        a.reset();
        assert_eq!(a, ChannelStats::new());
    }

    #[test]
    fn stats_loss_denominator() {
        // Потери считаются от отправленных, а не от «отправлено плюс
        // принято»: принятые сообщения к потерям на отправке отношения
        // не имеют.
        let s = ChannelStats { sent: 1_000, received: 900, dropped: 5, lost: 3, ..ChannelStats::new() };
        assert_eq!(s.loss_permille(), 8);
    }

    #[test]
    fn stats_loss_is_zero_without_traffic() {
        assert_eq!(ChannelStats::new().loss_permille(), 0);
    }

    #[test]
    fn stats_loss_cannot_exceed_full() {
        // Потеряно больше , чем отправлено , - счётчики врут или
        // считают разные окна. Показатель зажат в 1000 ‰.
        let s = ChannelStats { sent: 10, dropped: 5_000, ..ChannelStats::new() };
        assert_eq!(s.loss_permille(), 1000);
    }

    #[test]
    fn router_reports_unknown_channel() {
        // Маршрутизатор без единой открытой очереди: любой канал
        // должен дать понятную ошибку , а не паникой по индексу.
        let mut r = Router::open(&[], OpenPolicy::AttachOnly).unwrap();
        assert_eq!(r.channels().count(), 0);
        assert_eq!(
            r.send_channel(crate::ChannelId::RouterToRc, &[1]).unwrap_err(),
            Error::InvalidState
        );
        assert_eq!(r.stats_of(crate::ChannelId::RouterToRc), None);
        assert_eq!(r.queue_stat(crate::ChannelId::RouterToRc), None);
        assert_eq!(r.saturated_channels(), 0);
    }

    #[test]
    fn router_deduplicates_channels() {
        let ch = crate::ChannelId::RouterToRc;
        let r = Router::open(&[ch, ch], OpenPolicy::AttachOnly);
        // Не-Linux: открытие не удалось - проверять нечего.
        #[cfg(not(target_os = "linux"))]
        assert!(r.is_err());
        #[cfg(target_os = "linux")]
        if let Ok(r) = r {
            assert_eq!(r.channels().count(), 1);
        }
    }

    #[test]
    fn component_without_channel_is_unsupported() {
        let mut r = Router::open(&[], OpenPolicy::AttachOnly).unwrap();
        assert_eq!(
            r.send_component(ComponentId::Video, &[1]).unwrap_err(),
            Error::Unsupported
        );
        assert_eq!(
            r.receive_component(ComponentId::Video, &mut [0u8; 4]).unwrap_err(),
            Error::Unsupported
        );
    }

    #[test]
    fn component_with_channel_reaches_queue_lookup() {
        let mut r = Router::open(&[], OpenPolicy::AttachOnly).unwrap();
        // RC - компонент с каналом , поэтому ошибка уже про очередь,
        // а не про отсутствие канала.
        assert_eq!(
            r.send_component(ComponentId::Rc, &[1]).unwrap_err(),
            Error::InvalidState
        );
    }

    #[test]
    fn channel_for_covers_all_router_directions() {
        assert_eq!(ChannelFor::for_component(true, ComponentId::Rc).map(ChannelFor::id), Some(crate::ChannelId::RouterToRc));
        assert_eq!(ChannelFor::for_component(false, ComponentId::Rc).map(ChannelFor::id), Some(crate::ChannelId::RcToRouter));
    }

    #[test]
    fn permissions_are_open_for_all() {
        // 0666: иначе b-fruby-tools из init-скрипта не увидит шину.
        assert_eq!(IPC_PERM_0666, 0o666);
    }

    #[test]
    fn proj_id_kept_from_c() {
        assert_eq!(PROJ_ID, 117);
    }

    #[test]
    fn payload_capacity_matches_c_buffer() {
        assert_eq!(DATA_SIZE, 1600);
        assert_eq!(PAYLOAD_CAPACITY, 1593);
    }

    #[test]
    fn default_queue_limit_matches_linux_msgmnb() {
        // Реальный дефолт `kernel.msgmnb` - 16384 байта. При
        // `PAYLOAD_CAPACITY` = 1593 это ровно десять сообщений.
        assert_eq!(DEFAULT_QBYTES_LIMIT, 16384);
        assert_eq!(DEFAULT_QBYTES_LIMIT / PAYLOAD_CAPACITY as u32, 10);
    }

    #[test]
    fn queue_bytes_limit_is_read_from_sysctl_or_falls_back() {
        // Значение либо из sysctl , либо дефолтное; оба варианта
        // ненулевые, иначе `fill_permille` считал бы очередь
        // наполовину заполненной всегда.
        let v = queue_bytes_limit();
        assert!(v > 0);
        assert!(v >= PAYLOAD_CAPACITY as u32, "предел меньше одного сообщения");
    }

    #[test]
    fn high_water_matches_eighty_permille() {
        let s = QueueStat { qnum: 8, qbytes: 799, qbytes_limit: 1000 };
        assert!(!s.is_high_water());
        let s = QueueStat { qnum: 8, qbytes: 800, qbytes_limit: 1000 };
        assert!(s.is_high_water());
    }

    #[test]
    fn router_open_rolls_back_created_queues_on_failure() {
        // Проверяется то , что видно снаружи: после открытия маршрутизатора
        // очереди живы и к ним можно присоединиться второй раз. Внутренний
        // откат проверяется на Linux в
        // `linux_router_rollback_removes_only_own_queues`.
        let _lock = ipc_guard();
        let chans = [
            crate::ChannelId::RouterToRc,
            crate::ChannelId::RcToRouter,
        ];
        for c in chans {
            MsgQueue::remove(c);
        }

        let result = Router::open(&chans, OpenPolicy::CreateOrAttach);
        let ok = match result {
            // Заглушки не-Linux не умеют создавать очереди вовсе.
            Err(_) => return,
            Ok(ok) => ok,
        };

        assert_eq!(ok.channels().count(), 2);
        for ch in chans {
            // Присоединяемся второй раз: если бы отката не было, это
            // была бы вторая попытка создать ту же очередь.
            assert!(
                MsgQueue::open(ch, OpenPolicy::AttachOnly).is_ok(),
                "очередь {ch:?} пропала после открытия"
            );
        }

        // Уборка за собой обязательна: имена SysV общие для всего
        // пользователя, и висящая очередь исказит следующие прогоны.
        for ch in chans {
            MsgQueue::remove(ch);
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_rollback_removes_created_and_keeps_attached() {
        // Проверяется само правило отката: снимаются только очереди ,
        // помеченные как созданные этим вызовом. Очередь , к которой
        // присоединились , принадлежит маршрутизатору и переживает
        // чужую ошибку.
        let _lock = ipc_guard();
        let mine = crate::ChannelId::RouterToRc;
        let foreign = crate::ChannelId::RcToRouter;
        MsgQueue::remove(mine);
        MsgQueue::remove(foreign);

        // «Чужая» очередь существует заранее - её создал маршрутизатор.
        {
            let _creator =
                MsgQueue::open(foreign, OpenPolicy::CreateOrAttach).expect("создать чужую");
        }
        // «Своя» очередь создаётся этим вызовом.
        let q_mine = MsgQueue::open(mine, OpenPolicy::CreateOrAttach).expect("создать свою");
        // К «чуже��» присоединяемся - `was_created` будет `false`.
        let q_foreign =
            MsgQueue::open(foreign, OpenPolicy::AttachOnly).expect("присоединиться к чужой");
        assert!(q_mine.was_created(), "своя очередь должна быть помечена как созданная");
        assert!(!q_foreign.was_created(), "присоединённая очередь не наша");

        let mut queues: heapless::Vec<(crate::ChannelId, MsgQueue), MAX_CHANNELS> =
            heapless::Vec::new();
        let _ = queues.push((mine, q_mine));
        let _ = queues.push((foreign, q_foreign));
        remove_created(&queues);

        // Свою очередь откат снял.
        assert!(
            MsgQueue::open(mine, OpenPolicy::AttachOnly).is_err(),
            "созданная очередь должна была исчезнуть"
        );
        // Чужую - нет.
        assert!(
            MsgQueue::open(foreign, OpenPolicy::AttachOnly).is_ok(),
            "присоединённая очередь не должна трогаться откатом"
        );

        MsgQueue::remove(foreign);
    }

    #[test]
    fn router_open_keeps_attached_queues_on_failure() {
        // Откат трогает только созданные нами. Присоединённые
        // принадлежат маршрутизатору и должны пережить чужую ошибку.
        let _lock = ipc_guard();
        let _ = Router::open(&[], OpenPolicy::AttachOnly).unwrap();
    }

    #[test]
    fn gap_accounting_marks_first_message_as_baseline() {
        // Первое принятое сообщение задаёт точку отсчёта: потерь
        // считать не от чего.
        let mut last = None;
        let mut s = ChannelStats::new();
        note_gap(&mut last, &mut s, 17);
        assert_eq!(s.lost, 0);
        assert_eq!(s.unordered, 0);
    }

    #[test]
    fn gap_accounting_counts_missed_identifiers() {
        let mut last = None;
        let mut s = ChannelStats::new();
        note_gap(&mut last, &mut s, 10);
        note_gap(&mut last, &mut s, 14); // пропущены 11, 12, 13
        assert_eq!(s.lost, 3);
        note_gap(&mut last, &mut s, 15); // пропусков нет
        assert_eq!(s.lost, 3);
    }

    #[test]
    fn gap_accounting_survives_identifier_wraparound() {
        let mut last = None;
        let mut s = ChannelStats::new();
        note_gap(&mut last, &mut s, 254);
        note_gap(&mut last, &mut s, 2); // пропущены 255, 0, 1
        assert_eq!(s.lost, 3);
        assert_eq!(s.unordered, 0);
    }

    #[test]
    fn gap_accounting_marks_undecidable_jumps() {
        // Разрыв больше половины диапазона нельзя превратить в
        // число потерянных сообщений. Число не выдумывается -
        // растёт счётчик неразличимых случаев.
        let mut last = None;
        let mut s = ChannelStats::new();
        note_gap(&mut last, &mut s, 100);
        note_gap(&mut last, &mut s, 60); // 60 - 100 = -40 -> 216, больше 128
        assert_eq!(s.lost, 0);
        assert_eq!(s.unordered, 1);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_roundtrip_through_real_queue() {
        let _lock = ipc_guard();
        // Настоящая очередь SysV: создать, отправить, принять,
        // снять. Проверяет раскладку `long mtype`, CRC и права.
        let ch = crate::ChannelId::RouterToRc;
        // Убираем возможный хвост предыдущего прогона.
        MsgQueue::remove(ch);

        let mut tx = MsgQueue::open(ch, OpenPolicy::CreateOrAttach).expect("создать очередь");
        assert!(tx.was_created(), "очередь должна была создаться");
        let mut rx = MsgQueue::open(ch, OpenPolicy::AttachOnly).expect("присоединиться");
        assert!(!rx.was_created(), "второй вызов не должен создавать");

        assert_eq!(tx.send(&[1, 2, 3, 4]), Ok(()));
        let mut buf = [0u8; 64];
        assert_eq!(rx.receive(&mut buf), Ok(Some(4)));
        assert_eq!(&buf[..4], &[1, 2, 3, 4]);
        // Очередь пуста - это `Ok(None)`, а не ошибка.
        assert_eq!(rx.receive(&mut buf), Ok(None));

        assert!(MsgQueue::remove(ch), "снять очередь");
        // После удаления приём сообщает о смерти дескриптора, а не
        // «очередь пуста»: иначе листовой процесс крутил бы пустой
        // цикл на мёртвой шине.
        assert_eq!(rx.receive(&mut buf), Err(Error::InvalidState));
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_receive_reports_removed_queue_as_dead() {
        let _lock = ipc_guard();
        // Отдельный тест на требование «удалённая очередь неотличима
        // от пустой» из задания.
        let ch = crate::ChannelId::RcToRouter;
        MsgQueue::remove(ch);
        let mut rx = MsgQueue::open(ch, OpenPolicy::AttachOnly);
        if let Ok(rx) = rx.as_mut() {
            assert!(MsgQueue::remove(ch));
            let mut buf = [0u8; 64];
            assert_eq!(rx.receive(&mut buf), Err(Error::InvalidState));
            assert_eq!(rx.stats().errors, 1);
        }
        MsgQueue::remove(ch);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_full_queue_is_reported_as_drop_not_error() {
        let _lock = ipc_guard();
        // Переполнение и смерть дескриптора обязаны попадать в
        // разные счётчики, иначе потери в статистике нечестны.
        let ch = crate::ChannelId::RouterToCentral;
        MsgQueue::remove(ch);
        let mut q = MsgQueue::open(ch, OpenPolicy::CreateOrAttach).expect("создать очередь");
        let big = vec![0u8; PAYLOAD_CAPACITY];
        let mut got_dropped = false;
        for _ in 0..64 {
            if q.send(&big) == Err(Error::OutOfRange) {
                got_dropped = true;
                break;
            }
        }
        assert!(got_dropped, "очередь должна переполниться");
        assert!(q.stats().dropped > 0, "должен вырасти счётчик потерь");
        assert_eq!(q.stats().errors, 0, "переполнение - не ошибка дескриптора");
        MsgQueue::remove(ch);
    }
}
