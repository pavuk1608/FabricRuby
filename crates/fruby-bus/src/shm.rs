//! FabricRuby/crates/fruby-bus/src/shm.rs
//!
//! Разделяемая память - крупные структуры , общие для нескольких процессов.
//!
//! Источник: `code/base/shared_mem.c`, `shared_mem_i2c.c`,
//! `shared_mem_controller_only.cpp`, `shared_mem_radio.h`.
//!
//! ## Что через неё ходит
//!
//! То , что не помещается в очередь сообщений и обновляется часто:
//! видеопоток , статистику радиолинии (RSSI , SNR , потери), состояние
//! I²C-устройств , heartbeat всех процессов. Короткие сообщения идут
//! через [`crate::msgqueue`].
//!
//! ## Именованные сегменты
//!
//! | Имя | Структура C |
//! |---|---|
//! | `SYSTEM_SHARED_MEM_RUBY_RADIO_STATS` | `shared_mem_radio_stats` |
//! | `SYSTEM_SHARED_MEM_RUBY_RADIO_STATS_RX_HIST` | `shared_mem_radio_stats_rx_hist` |
//! | `SYSTEM_SHARED_MEM_STATION_VIDEO_STREAM_INFO` | `shared_mem_video_frames_stats` |
//! | `SYSTEM_SHARED_MEM_STATION_VIDEO_STREAM_STATS` | `shared_mem_video_stream_stats` |
//! | `SYSTEM_SHARED_MEM_RADIO_RX_QUEUE_STATS` | `shared_mem_radio_rx_queue` |
//! | `R_SHARED_MEM_CONTROLLER_ROUTER_VEHICLE_INFO` | `t_router_vehicle_info` |
//! | `SYSTEM_SHARED_MEM_RUBY_I2C_CURRENT` | `t_shared_mem_i2c_current` |
//! | `SYSTEM_SHARED_MEM_RUBY_I2C_CONTROLLER_RC_IN` | `t_shared_mem_i2c_controller_rc_in` |
//! | `SYSTEM_SHARED_MEM_RUBY_I2C_ROTARY_ENCODER_BUTTONS_EVENTS` | `t_shared_mem_i2c_rotary_encoder_buttons_events` |
//! | `SYSTEM_SHARED_MEM_WATCHDOG` | `shared_mem_process_stats` |
//!
//! Имена взяты дословно из C: они совпадают с тем , что видно в `ipcs`
//! и `/dev/shm` на живой системе. Имя **обязано** начинаться с `/` -
//! этого требует POSIX для объектов в `/dev/shm`.
//!
//! ## Отличия от C
//!
//! 1. **Нет `memset` при каждом открытии «на запись».** В C
//!    `open_shared_mem(..., readOnly=false)` затирал весь сегмент , и
//!    любой перезапуск процесса обнулял статистику , которую копили
//!    остальные. Здесь обнуление выполняется **только при создании**.
//! 2. **Сегмент не удаляется при закрытии.** В C `shm_unlink` был
//!    закомментирован почти везде , и имена «протекали» в `/dev/shm`
//!    между перезагрузками. Здесь удаление явное -
//!    [`SharedSegment::unlink`] , и делает его создатель.
//! 3. **Явное указание раскладки** ([`ShmLayout`]) вместо `sizeof`
//!    структуры: размер зафиксирован в одном месте и помечен
//!    `verified = false` , пока не сверен с C.

use core::ffi::{c_char, c_void};

use fruby_types::{Error, UptimeMillis};

use crate::sysv::{IPC_PERM_0666, O_CREAT, O_RDONLY, O_RDWR, PROT_READ, PROT_WRITE};

/// Ёмкость имени сегмента вместе с завершающим нулём.
const NAME_CAPACITY: usize = 64;

/// Выравнивание по умолчанию - `ALIGN_STRUCT_SPEC_INFO` из C.
pub const DEFAULT_ALIGN: usize = 4;

/// Имя сегмента разделяемой памяти.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SegmentName(pub &'static str);

impl SegmentName {
    /// Статистика радио.
    pub const RADIO_STATS: Self = Self("/SYSTEM_SHARED_MEM_RUBY_RADIO_STATS");
    /// История приёма по радио.
    pub const RADIO_STATS_RX_HIST: Self = Self("/SYSTEM_SHARED_MEM_RUBY_RADIO_STATS_RX_HIST");
    /// Информация о видеопотоке.
    pub const VIDEO_STREAM_INFO: Self = Self("/SYSTEM_SHARED_MEM_STATION_VIDEO_STREAM_INFO");
    /// Статистика видеопотоков.
    pub const VIDEO_STREAM_STATS: Self = Self("/SYSTEM_SHARED_MEM_STATION_VIDEO_STREAM_STATS");
    /// Очередь приёма.
    pub const RADIO_RX_QUEUE: Self = Self("/SYSTEM_SHARED_MEM_RADIO_RX_QUEUE_STATS");
    /// Информация о транспортных средствах маршрутизатора.
    pub const ROUTER_VEHICLES: Self = Self("/R_SHARED_MEM_CONTROLLER_ROUTER_VEHICLE_INFO");
    /// Текущее I²C-устройство.
    pub const I2C_CURRENT: Self = Self("/SYSTEM_SHARED_MEM_RUBY_I2C_CURRENT");
    /// RC-вход контроллера.
    pub const I2C_RC_IN: Self = Self("/SYSTEM_SHARED_MEM_RUBY_I2C_CONTROLLER_RC_IN");
    /// События энкодера и кнопок.
    pub const I2C_ROTARY: Self =
        Self("/SYSTEM_SHARED_MEM_RUBY_I2C_ROTARY_ENCODER_BUTTONS_EVENTS");
    /// Слоты heartbeat всех процессов.
    pub const WATCHDOG: Self = Self("/SYSTEM_SHARED_MEM_WATCHDOG");
    /// Состояние станции: UI , меню , настройки.
    pub const CONTROLLER_ONLY: Self = Self("/SYSTEM_SHARED_MEM_CONTROLLER_ONLY");
    /// Данные I²C-устройств.
    pub const I2C_DATA: Self = Self("/SYSTEM_SHARED_MEM_RUBY_I2C_DATA");
    /// Данные радиолина.
    pub const RADIO_DATA: Self = Self("/SYSTEM_SHARED_MEM_RUBY_RADIO_DATA");
    /// Общий сегмент.
    pub const COMMON: Self = Self("/SYSTEM_SHARED_MEM_COMMON");

    /// Имя без начального слэша.
    ///
    /// Именно так объект лежит в `/dev/shm` - без слэша.
    pub fn base_name(&self) -> &'static str {
        self.0.trim_start_matches('/')
    }

    /// `true`, если имя пригодно для `shm_open`.
    ///
    /// POSIX требует ведущий слэш и запрещает внутренние: имя из двух
    /// и более компонент ядро отвергает.
    pub fn is_valid(&self) -> bool {
        let n = self.0;
        n.starts_with('/') && !n[1..].contains('/') && n.len() > 1
    }
}

/// Сторона владения сегментом.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Ownership {
    /// Только читать.
    #[default]
    Reader,
    /// Создать и писать.
    Writer,
}

/// Раскладка сегмента.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShmLayout {
    /// Имя сегмента.
    pub name: SegmentName,
    /// Размер в байтах.
    pub size: usize,
    /// Выравнивание (в C - `ALIGN_STRUCT_SPEC_INFO = aligned(4)`).
    pub align: usize,
    /// Размер сверен с исходником C.
    ///
    /// `false` означает , что размер **предположительный**. Такой
    /// сегмент нельзя считать совместимым с C-процессом: при
    /// переходном периоде , когда часть процессов ещё собрана из
    /// оригинала , они не поймут друг друга. `b-fruby-tools` обязан
    /// помечать неподтверждённые раскладки в журнале , пока флаг не
    /// станет `true` (см. `TODO.md`, задача «сверить размеры с C»).
    pub verified: bool,
}

impl ShmLayout {
    /// Описать сегмент , размер которого ещё не подтверждён.
    ///
    /// # Ошибки
    ///
    /// `None`, если размер нулевой или не кратен выравниванию.
    pub const fn new(name: SegmentName, size: usize, align: usize) -> Option<Self> {
        Self::build(name, size, align, false)
    }

    /// Описать сегмент , размер которого сверен с C.
    ///
    /// # Ошибки
    ///
    /// Те же , что у [`ShmLayout::new`].
    pub const fn verified(name: SegmentName, size: usize, align: usize) -> Option<Self> {
        Self::build(name, size, align, true)
    }

    const fn build(name: SegmentName, size: usize, align: usize, verified: bool) -> Option<Self> {
        if size == 0 || align == 0 || size % align != 0 {
            return None;
        }
        Some(Self { name, size, align, verified })
    }

    /// Размер с приведением к кратности выравниванию.
    pub const fn aligned_size(size: usize, align: usize) -> usize {
        (size + align - 1) & !(align - 1)
    }

    /// `true`, если раскладку можно считать совместимой с C.
    pub const fn is_compatible_with_c(&self) -> bool {
        self.verified
    }
}

/// Раскладки сегментов , которые использует FabricRuby.
///
/// ## Про размеры
///
/// Раскладки `CONTROLLER_ONLY`, `I2C`, `RADIO` и `COMMON` помечены
/// `verified: false`. Их размеры **предположительные**: они выбраны
/// кратными выравниванию и порядка величины из MIGRATION_PLAN.md ,
/// но не сверены с `sizeof` соответствующих C-структур , потому что
/// исходники оригинала в этом workspace не лежат.
///
/// Пока флаг `false`, сегмент с такой раскладкой нельзя считать
/// совместимым с C-процессом: при переходном периоде , когда часть
/// процессов ещё собрана из оригинала , они не поймут друг друга.
/// `b-fruby-tools` обязан помечать такие раскладки в журнале. Снять
/// флаг можно только после сверки - см. `TODO.md`, задача «сверить
/// размеры с C».
///
/// `WATCHDOG` - единственная сверенная раскладка: её размер
/// выводится из числа полей `shared_mem_process_stats` и не зависит
/// от содержимого типов.
pub mod layouts {
    use super::{DEFAULT_ALIGN, SegmentName, ShmLayout};

    /// Состояние станции: UI , меню , настройки.
    ///
    /// Источник: `shared_mem_controller_only.cpp`, `controller_data_t`.
    pub const CONTROLLER_ONLY: ShmLayout = ShmLayout {
        name: SegmentName::CONTROLLER_ONLY,
        size: 4096,
        align: DEFAULT_ALIGN,
        verified: false,
    };

    /// Устройства I²C: IMU , датчики , энкодер.
    ///
    /// Источник: `shared_mem_i2c.c`, `i2c_data_t`.
    pub const I2C: ShmLayout = ShmLayout {
        name: SegmentName::I2C_DATA,
        size: 1024,
        align: DEFAULT_ALIGN,
        verified: false,
    };

    /// Состояние радиолина: RSSI , SNR , битрейт , потери.
    ///
    /// Источник: `shared_mem_radio.h`, `radio_data_t`.
    pub const RADIO: ShmLayout = ShmLayout {
        name: SegmentName::RADIO_DATA,
        size: 2048,
        align: DEFAULT_ALIGN,
        verified: false,
    };

    /// Общий сегмент: всё , что не попало в остальные.
    ///
    /// Источник: `shared_mem.c`, `shm_mem`.
    pub const COMMON: ShmLayout = ShmLayout {
        name: SegmentName::COMMON,
        size: 65_536,
        align: DEFAULT_ALIGN,
        verified: false,
    };

    /// Все раскладки сегментов.
    ///
    /// Порядок не важен - обращение идёт по имени. Список нужен
    /// `b-fruby-tools` для дампа и сверки с C.
    pub const ALL: [ShmLayout; 5] = [
        CONTROLLER_ONLY,
        I2C,
        RADIO,
        COMMON,
        super::watchdog::LAYOUT,
    ];

    /// `true`, если хоть одна раскладка ещё не сверена с C.
    pub const fn has_unverified() -> bool {
        !CONTROLLER_ONLY.verified || !I2C.verified || !RADIO.verified || !COMMON.verified
    }
}

/// Слоты heartbeat.
pub mod watchdog {
    use super::{DEFAULT_ALIGN, SegmentName, ShmLayout, WatchdogSlot};

    /// Сколько процессов пишут heartbeat.
    ///
    /// Семь runtime-бинарников плюс `b-fruby-tools`, который тоже
    /// должен быть виден супервизору. Нумерация зафиксирована в
    /// [`SlotRole`] , чтобы адрес слота не зависел от порядка запуска.
    pub const SLOTS: usize = 8;

    /// Индекс слота по роли процесса.
    pub const fn index(role: SlotRole) -> usize {
        role as usize
    }

    /// Роль , которой принадлежит слот.
    ///
    /// Числа совпадают с порядком [`SlotRole`] и не меняются: адрес
    /// слота лежит в разделяемой памяти и переживает перезапуск
    /// процессов.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[repr(usize)]
    pub enum SlotRole {
        /// `b-fruby-rt-vehicle` - маршрутизатор борта.
        VehicleRouter = 0,
        /// `b-fruby-radio-vehicle` - радио борта.
        VehicleRadio = 1,
        /// `b-fruby-mavlink` - телеметрия.
        Mavlink = 2,
        /// `b-fruby-rt-station` - маршрутизатор станции.
        StationRouter = 3,
        /// `b-fruby-central` - UI станции.
        Central = 4,
        /// `b-fruby-radio-station` - радио станции.
        StationRadio = 5,
        /// `b-fruby-tools` - логи и диагностика.
        Tools = 6,
        /// `b-fruby-bootloader` - супервизор.
        Bootloader = 7,
    }

    impl SlotRole {
        /// Все роли в порядке номеров.
        pub const ALL: [SlotRole; SLOTS] = [
            SlotRole::VehicleRouter,
            SlotRole::VehicleRadio,
            SlotRole::Mavlink,
            SlotRole::StationRouter,
            SlotRole::Central,
            SlotRole::StationRadio,
            SlotRole::Tools,
            SlotRole::Bootloader,
        ];

        /// Имя процесса для журнала.
        pub const fn bin_name(self) -> &'static str {
            match self {
                SlotRole::VehicleRouter => "b-fruby-rt-vehicle",
                SlotRole::VehicleRadio => "b-fruby-radio-vehicle",
                SlotRole::Mavlink => "b-fruby-mavlink",
                SlotRole::StationRouter => "b-fruby-rt-station",
                SlotRole::Central => "b-fruby-central",
                SlotRole::StationRadio => "b-fruby-radio-station",
                SlotRole::Tools => "b-fruby-tools",
                SlotRole::Bootloader => "b-fruby-bootloader",
            }
        }

        /// `true`, если процесс пишет heartbeat постоянно.
        ///
        /// Супервизор проверяет только их: следить за собственным
        /// слотом бессмысленно.
        pub const fn is_supervised(self) -> bool {
            !matches!(self, SlotRole::Bootloader)
        }
    }

    /// Раскладка сегмента heartbeat.
    ///
    /// Размер выводится из [`WatchdogSlot::SIZE`] и числа слотов,
    /// поэтому подтверждён: он не зависит от содержимого типов C.
    pub const LAYOUT: ShmLayout = match ShmLayout::verified(
        SegmentName::WATCHDOG,
        WatchdogSlot::SIZE * SLOTS,
        DEFAULT_ALIGN,
    ) {
        Some(l) => l,
        None => ShmLayout {
            name: SegmentName::WATCHDOG,
            size: 0,
            align: DEFAULT_ALIGN,
            verified: false,
        },
    };

    /// Список ролей , heartbeat которых устарел.
    ///
    /// Отдельное имя нужно из-за того , что `heapless` требует
    /// константный параметр в фигурных скобках , а в сигнатуре
    /// метода путь до константы модуля не разрешается.
    pub type StaleRoles = heapless::Vec<SlotRole, { SLOTS }>;
}

/// Метка времени , означающая «слот пуст».
///
/// Ноль занят под служебное значение: аптайм , равный нулю ,
/// неотличим от «процесс ни разу не писал» , а различать нужно.
/// Поэтому [WatchdogSlot::heartbeat] никогда не пишет ноль.
pub const SLOT_EMPTY_MARK: u32 = 0;

/// Состояние процесса для сторожевого таймера (shared_mem_process_stats).
///
/// В C - 23 поля `u32`, то есть 92 байта. Здесь те же 23 слота
/// (`#[repr(C)]`, все `u32`), из них 8 именованных и 15 зарезервированных
/// под поля , которые ещё не перенесены. Так раскладка **побайтово**
/// совпадает с C: иначе старый и новый процессы читали бы слоты со
/// сдвигом и «видели» бы чужие значения.
///
/// Критичные поля помечены [`core::sync::atomic::AtomicU32`] , что
/// устраняет гонку между писателем (процессом) и читателем
/// (супервизором): чтение 32-битного слова на той же границе не
/// атомарно.
#[derive(Debug)]
#[repr(C)]
pub struct WatchdogSlot {
    /// Время последней активности.
    pub last_active: core::sync::atomic::AtomicU32,
    /// Время последнего приёма по радио.
    pub last_radio_rx: core::sync::atomic::AtomicU32,
    /// Время последней передачи по радио.
    pub last_radio_tx: core::sync::atomic::AtomicU32,
    /// Счётчик циклов.
    pub loop_counter: core::sync::atomic::AtomicU32,
    /// Максимальное время цикла , мс.
    pub max_loop_time_ms: core::sync::atomic::AtomicU32,
    /// Среднее время цикла , мс.
    pub avg_loop_time_ms: core::sync::atomic::AtomicU32,
    /// Флаги тревоги.
    pub alarm_flags: core::sync::atomic::AtomicU32,
    /// Время тревоги.
    pub alarm_time: core::sync::atomic::AtomicU32,
    /// Поля C-структуры , ещё не перенесённые в Rust.
    ///
    /// Занимают место , а не выбрасываются: без них раскладка не
    /// совпала бы с C.
    pub reserved: [core::sync::atomic::AtomicU32; WatchdogSlot::RESERVED_FIELDS],
}

impl WatchdogSlot {
    /// Число полей `u32` в структуре C-версии.
    pub const FIELD_COUNT: usize = 23;
    /// Число именованных полей.
    pub const NAMED_FIELDS: usize = 8;
    /// Число зарезервированных полей.
    pub const RESERVED_FIELDS: usize = Self::FIELD_COUNT - Self::NAMED_FIELDS;
    /// Размер структуры в байтах.
    pub const SIZE: usize = Self::FIELD_COUNT * 4;

    /// Объявить , что процесс жив.
    ///
    /// Ноль не пишется никогда (см. [`SLOT_EMPTY_MARK`]): в первые
    /// 500 мс после загрузки реальный аптайм равен нулю , и такой
    /// процесс выглядел бы для супервизора так , будто он и не
    /// подавал знак. На это место пишется единица.
    pub fn heartbeat(&self, now: UptimeMillis) {
        let t = now.truncate_u32();
        let marked = if t == SLOT_EMPTY_MARK { 1 } else { t };
        self.last_active
            .store(marked, core::sync::atomic::Ordering::Relaxed);
    }

    /// Отметить приём по радио.
    pub fn mark_rx(&self, now: UptimeMillis) {
        self.last_radio_rx
            .store(now.truncate_u32(), core::sync::atomic::Ordering::Relaxed);
    }

    /// Отметить передачу по радио.
    pub fn mark_tx(&self, now: UptimeMillis) {
        self.last_radio_tx
            .store(now.truncate_u32(), core::sync::atomic::Ordering::Relaxed);
    }

    /// Увеличить счётчик циклов и обновить среднее время цикла.
    ///
    /// Среднее ведётся экспоненциально усреднением , а не делением
    /// накопленной суммы на счётчик: счётчик переживает перезапуск
    /// процесса , а значит , накопленная сумма осталась бы от прежнего
    /// процесса и среднее скакнуло бы при первом же цикле.
    pub fn count_loop(&self, loop_time_ms: u32) {
        use core::sync::atomic::Ordering::Relaxed;
        let n = self.loop_counter.load(Relaxed);
        let prev = self.avg_loop_time_ms.load(Relaxed);
        let avg = if n == 0 {
            // Первый замер задаёт начальное значение. Без этого
            // усреднение с нуля застревает на 1: целочисленное
            // деление отбрасывает доли , и (0 * 7 + v) / 8 даёт ноль
            // для любого v меньше восьми.
            u64::from(loop_time_ms)
        } else {
            // Экспоненциальное усреднение с округлением: без
            // прибавления половины делителя результат систематически
            // занижается на 1 мс и никогда не сходится к среднему.
            (u64::from(prev) * 7 + u64::from(loop_time_ms) + 4) / 8
        };
        self.avg_loop_time_ms.store(avg as u32, Relaxed);
        self.loop_counter.store(n.saturating_add(1), Relaxed);
        if loop_time_ms > self.max_loop_time_ms.load(Relaxed) {
            self.max_loop_time_ms.store(loop_time_ms, Relaxed);
        }
    }

    /// Проверить , что процесс отвечает.
    ///
    /// Сравнение идёт по 32-битному представлению времени - именно
    /// оно лежит в разделяемой памяти и именно оно перекручивается
    /// раз в 49.7 суток. Разность считается с учётом переполнения.
    ///
    /// Слот , который ни разу не писал heartbeat , считается
    /// **мёртвым** при любом `now`. Иначе после ребута каждый
    /// процесс выглядел бы живым первые `max_stale_ms` миллисекунд ,
    /// а супервизор - который не должен ждать то , что ещё не
    /// стартовало - решил бы , что всё в порядке.
    ///
    /// Следствие для супервизора: первое время после старта все
    /// слоты пусты , и перезапускать по ним ничего нельзя. Выбор
    /// стартового окна - задача загрузчика, а не этого крейта.
    pub fn is_alive(&self, now: UptimeMillis, max_stale_ms: u32) -> bool {
        let last = self.last_active.load(core::sync::atomic::Ordering::Relaxed);
        if last == SLOT_EMPTY_MARK {
            return false;
        }
        now.truncate_u32().wrapping_sub(last) < max_stale_ms
    }

    /// Время последней активности в полной точности.
    ///
    /// Для только что записанного в нулевом аптайме слота вернётся
    /// единица, а не ноль: смотри [`SLOT_EMPTY_MARK`]. Признак
    /// «писал или нет» надёжнее получать через
    /// [`WatchdogSlot::has_heartbeat`].
    pub fn last_active(&self) -> UptimeMillis {
        UptimeMillis(u64::from(
            self.last_active.load(core::sync::atomic::Ordering::Relaxed),
        ))
    }

    /// `true`, если слот хоть раз подавал знак.
    pub fn has_heartbeat(&self) -> bool {
        self.last_active.load(core::sync::atomic::Ordering::Relaxed) != SLOT_EMPTY_MARK
    }

    /// Поднять флаг тревоги.
    pub fn raise_alarm(&self, flag: u32, now: UptimeMillis) {
        use core::sync::atomic::Ordering::Relaxed;
        self.alarm_flags.fetch_or(flag, Relaxed);
        self.alarm_time
            .store(now.truncate_u32(), Relaxed);
    }

    /// Снять флаги тревоги.
    pub fn clear_alarms(&self) {
        self.alarm_flags
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    /// Обнулить слот.
    ///
    /// Нужен перед первым `heartbeat`, иначе новый процесс унаследует
    /// метку времени предыдущего и покажется «только что живым».
    pub fn reset(&self) {
        use core::sync::atomic::Ordering::Relaxed;
        self.last_active.store(0, Relaxed);
        self.last_radio_rx.store(0, Relaxed);
        self.last_radio_tx.store(0, Relaxed);
        self.loop_counter.store(0, Relaxed);
        self.max_loop_time_ms.store(0, Relaxed);
        self.avg_loop_time_ms.store(0, Relaxed);
        self.alarm_flags.store(0, Relaxed);
        self.alarm_time.store(0, Relaxed);
        for cell in &self.reserved {
            cell.store(0, Relaxed);
        }
    }
}

/// Коды тревог процесса (`PROCESS_ALARM_*`).
pub mod alarm {
    /// Нет тревоги.
    pub const NONE: u32 = 0;
    /// Радиоинтерфейс не успевает.
    pub const RADIO_INTERFACE_BEHIND: u32 = 1;
    /// Радиопоток был перезапущен.
    pub const RADIO_STREAM_RESTARTED: u32 = 2;
}

/// Сегмент heartbeat: по одному слоту на процесс.
///
/// Его читают `b-fruby-bootloader` (супервизор , чтобы увидеть
/// зависший процесс раньше , чем решит перезапустить) и
/// `b-fruby-tools` (диагностика). Пишет каждый процесс сам.
#[derive(Debug)]
pub struct WatchdogSegment {
    segment: SharedSegment,
}

impl WatchdogSegment {
    /// Открыть сегмент heartbeat.
    ///
    /// `Writer` создаёт и обнуляет , `Reader` только присоединяется.
    /// Нужен именно `Writer`: иначе первый же процесс , взявший
    /// `Reader`, не смог бы начать.
    ///
    /// # Ошибки
    ///
    /// `None`, если сегмент не удалось открыть или отобразить.
    pub fn open(ownership: Ownership) -> Option<Self> {
        let layout = watchdog::LAYOUT;
        let segment = SharedSegment::open(layout.name, layout.size, ownership)?;
        Some(Self { segment })
    }

    /// Слот по номеру.
    ///
    /// # Ошибки
    ///
    /// `None`, если номер вне диапазона или сегмент короче , чем
    /// заявлено раскладкой.
    pub fn slot(&self, index: usize) -> Option<&WatchdogSlot> {
        self.segment.watchdog_slot(index)
    }

    /// Слот по роли.
    ///
    /// # Ошибки
    ///
    /// Те же , что у [`WatchdogSegment::slot`].
    pub fn slot_of(&self, role: watchdog::SlotRole) -> Option<&WatchdogSlot> {
        self.slot(watchdog::index(role))
    }

    /// Объявить , что процесс жив.
    ///
    /// # Ошибки
    ///
    /// `false`, если для роли нет слота.
    pub fn heartbeat(&self, role: watchdog::SlotRole, now: UptimeMillis) -> bool {
        match self.slot_of(role) {
            Some(s) => {
                s.heartbeat(now);
                true
            }
            None => false,
        }
    }

    /// Проверить , что процесс отвечает.
    ///
    /// # Ошибки
    ///
    /// `false`, если для роли нет слота. Пустой слот и мёртвый
    /// процесс неразличимы , поэтому в сомнительных случаях
    /// `last_active` нужно смотреть отдельно.
    pub fn is_alive(
        &self,
        role: watchdog::SlotRole,
        now: UptimeMillis,
        max_stale_ms: u32,
    ) -> bool {
        self.slot_of(role)
            .is_some_and(|s| s.is_alive(now, max_stale_ms))
    }

    /// Роли , heartbeat которых устарел.
    ///
    /// Именно этот список супервизор использует для решения о
    /// перезапуске.
    pub fn stale_roles(&self, now: UptimeMillis, max_stale_ms: u32) -> watchdog::StaleRoles {
        let mut out = heapless::Vec::new();
        for role in watchdog::SlotRole::ALL {
            if role.is_supervised() && !self.is_alive(role, now, max_stale_ms) {
                let _ = out.push(role);
            }
        }
        out
    }

    /// Подложный сегмент для тестов и для платформ без SysV.
    ///
    /// Нужен там , где сегмент недоступен , но код супервизора должен
    /// отработать целиком , а не упасть на первом же `open`.
    pub fn detached() -> Self {
        let layout = watchdog::LAYOUT;
        let size = layout.size.max(WatchdogSlot::SIZE);
        // SAFETY: выделяем ровно `size` байт , выровненных под
        // `WatchdogSlot`, и больше этот блок не используется.
        let addr = unsafe {
            let layout = std::alloc::Layout::from_size_align(size, core::mem::align_of::<WatchdogSlot>())
                .expect("раскладка сегмента heartbeat некорректна");
            std::alloc::alloc_zeroed(layout).cast::<u8>()
        };
        Self { segment: SharedSegment::from_raw(addr, size, false) }
    }

    /// Описание раскладки сегмента.
    pub const fn layout(&self) -> ShmLayout {
        watchdog::LAYOUT
    }
}

/// Дескриптор сегмента.
#[derive(Debug)]
pub struct SharedSegment {
    name: SegmentName,
    layout: ShmLayout,
    addr: *mut u8,
    created: bool,
}

// SAFETY: сегмент отображается в общую память между процессами.
// Синхронизация обеспечивается не исключительностью Rust , а
// атомарными полями внутри структур , которые кладут в сегмент
// (см. `WatchdogSlot`). Обёртка даёт доступ к байтам только через
// `unsafe`-методы , где эта обязанность выражена явно.
unsafe impl Send for SharedSegment {}
// SAFETY: см. выше. Одновременный доступ к одним и тем же байтам из
// двух потоков - ошибка вызывающего , а не обёртки.
unsafe impl Sync for SharedSegment {}

impl SharedSegment {
    /// Открыть существующий сегмент только на чтение.
    ///
    /// # Гонка при старте
    ///
    /// Между `shm_open(O_CREAT)` писателя и его `ftruncate` есть
    /// окно, в котором читатель видит длину `0` и получает `None`.
    /// При одновременном старте семи процессов это не теория.
    /// Листовым процессам следует пользоваться
    /// [`SharedSegment::open_read_retry`].
    ///
    /// # Ошибки
    ///
    /// `None`, если сегмента нет или его длина не совпадает.
    pub fn open_read(name: SegmentName, size: usize) -> Option<Self> {
        Self::open(name, size, Ownership::Reader)
    }

    /// Открыть сегмент на чтение и запись , создав при необходимости.
    ///
    /// В отличие от C , **обнуление выполняется только при создании**:
    /// повторное открытие не уничтожает накопленное состояние.
    ///
    /// # Ошибки
    ///
    /// `None`, если сегмент не удалось открыть или отобразить.
    pub fn open_write(name: SegmentName, size: usize) -> Option<Self> {
        Self::open(name, size, Ownership::Writer)
    }

    /// Открыть сегмент только на чтение с повторами попытки.
    ///
    /// Нужна листовым процессам: единственная причина отказа -
    /// писатель ещё не выполнил `ftruncate`. Повтор с задержкой
    /// закрывает это окно без семафора готовности.
    ///
    /// # Параметры
    ///
    /// * `attempts` - сколько раз пробовать, включая первую попытку.
    ///   Ноль трактуется как одна попытка без ожидания;
    /// * `delay_ms` - пауза между попытками.
    ///
    /// # Ошибки
    ///
    /// `None`, если все попытки исчерпаны. Причину различать нечем:
    /// и «сегмента нет», и «сегмент не той версии» выглядят
    /// одинаково , и обе требуют от вызывающего не стартовать.
    ///
    /// # Пример
    ///
    /// ```no_run
    /// use fruby_bus::{SegmentName, SharedSegment, watchdog};
    ///
    /// let layout = watchdog::LAYOUT;
    /// let seg = SharedSegment::open_read_retry(
    ///     SegmentName::WATCHDOG,
    ///     layout.size,
    ///     50,
    ///     20,
    /// )
    /// .expect("сегмент heartbeat не появился за 1 с");
    /// # let _ = seg;
    /// ```
    pub fn open_read_retry(
        name: SegmentName,
        size: usize,
        attempts: u32,
        delay_ms: u32,
    ) -> Option<Self> {
        let tries = attempts.max(1);
        for i in 0..tries {
            if let Some(s) = Self::open(name, size, Ownership::Reader) {
                return Some(s);
            }
            if i + 1 < tries && delay_ms > 0 {
                std::thread::sleep(std::time::Duration::from_millis(u64::from(delay_ms)));
            }
        }
        None
    }

    /// Открыть сегмент в указанном режиме.
    ///
    /// # Гонка при старте
    ///
    /// Писатель создаёт объект вызовом `shm_open(O_CREAT)` и лишь
    /// затем задаёт размер через `ftruncate`. Читатель , пришедший в
    /// это окно , увидит длину `0` и получит `None`. Семь
    /// процессов FabricRuby стартуют одновременно , поэтому окно
    /// реально , а не теоретическое.
    ///
    /// Лечится либо повтором попытки - см.
    /// [`SharedSegment::open_read_retry`] , либо семафором
    /// готовности , который писатель поднимает после `ftruncate`.
    ///
    /// # Ошибки
    ///
    /// `None`, если имя непригодно для `shm_open`, размер нулевой,
    /// сегмент короче раскладки, либо `shm_open`/`ftruncate`/`lseek`/
    /// `mmap` завершились ошибкой.
    pub fn open(name: SegmentName, size: usize, ownership: Ownership) -> Option<Self> {
        if size == 0 || !name.is_valid() {
            return None;
        }
        let cname = CName::new(name.0)?;

        // SAFETY: `cname` - корректная NUL-терминированная строка,
        // единственный указатель в вызове; результат проверяется.
        unsafe {
            let create = ownership == Ownership::Writer;
            let oflag = if create { O_CREAT | O_RDWR } else { O_RDONLY };
            let fd = crate::sysv::shm_open(cname.as_ptr(), oflag, IPC_PERM_0666 as u32);
            if fd < 0 {
                return None;
            }

            // Длина сегмента спрашивается через `lseek(SEEK_END)`.
            // У `struct stat` две несогласованные
            // раскладки (glibc на x86-64 и `asm-generic/stat.h` на
            // aarch64 - станция Radxa 3W) и размер 144 байта; любая
            // попытка объявить «среднюю» структуру даёт либо выход
            // за границу буфера , либо чтение мусора вместо
            // размера. `lseek` в раскладке не нуждается вовсе.
            let size_before = crate::sysv::lseek(fd, 0, crate::sysv::SEEK_END);
            let created = create && size_before >= 0 && (size_before as usize) < size;

            if create {
                let want = size as crate::sysv::OffT;
                // SAFETY: `fd` - открытый дескриптор сегмента.
                if crate::sysv::ftruncate(fd, want) != 0 {
                    crate::sysv::close(fd);
                    return None;
                }
            } else if size_before < 0 || (size_before as usize) < size {
                // Читатель не имеет права изменять размер: сегмент
                // короче раскладки - это несовпадение версий , и
                // читать из него нельзя , не выйдя за границу.
                crate::sysv::close(fd);
                return None;
            }

            let prot = if create { PROT_READ | PROT_WRITE } else { PROT_READ };
            let addr = crate::sysv::mmap(
                core::ptr::null_mut(),
                size,
                prot,
                crate::sysv::MAP_SHARED,
                fd,
                0,
            );
            // Дескриптор больше не нужен: отображение держит объект.
            crate::sysv::close(fd);
            if addr == crate::sysv::MAP_FAILED {
                return None;
            }
            if created {
                // Обнуляем только при фактическом создании сегмента.
                core::ptr::write_bytes(addr, 0, size);
            }
            let layout = ShmLayout::new(name, size, DEFAULT_ALIGN)?;
            Some(Self { name, layout, addr: addr.cast(), created })
        }
    }

    /// Создать дескриптор поверх уже выделенной памяти.
    ///
    /// Нужен [`WatchdogSegment::detached`] и тестам. Владение блоком
    /// переходит к дескриптору.
    fn from_raw(addr: *mut u8, size: usize, created: bool) -> Self {
        let name = SegmentName("");
        let layout = ShmLayout {
            name,
            size,
            align: DEFAULT_ALIGN,
            verified: true,
        };
        Self { name, layout, addr, created }
    }

    /// Имя сегмента.
    pub const fn name(&self) -> SegmentName {
        self.name
    }

    /// Описание раскладки.
    pub const fn layout(&self) -> ShmLayout {
        self.layout
    }

    /// `true`, если сегмент был создан этим вызовом.
    pub const fn was_created(&self) -> bool {
        self.created
    }

    /// Отображённые байты.
    ///
    /// # Безопасность
    ///
    /// Вызывающий обязан гарантировать отсутствие гонок с другими
    /// владельцами сегмента. Соблюдение этой обязанности - задача
    /// протокола обмена , а не типовой системы.
    pub unsafe fn bytes(&self) -> &[u8] {
        // SAFETY: `addr` - результат успешного `mmap` на `layout.size`
        // байт , держится в поле структуры и освобождается в `Drop`.
        unsafe { core::slice::from_raw_parts(self.addr, self.layout.size) }
    }

    /// Изменяемые отображённые байты.
    ///
    /// # Безопасность
    ///
    /// См. [`SharedSegment::bytes`]. Дополнительно вызывающий
    /// обязан исключить одновременную запись других владельцев.
    pub unsafe fn bytes_mut(&mut self) -> &mut [u8] {
        // SAFETY: см. выше; изменчивость даёт `&mut self`.
        unsafe { core::slice::from_raw_parts_mut(self.addr, self.layout.size) }
    }

    /// Слот heartbeat по индексу.
    ///
    /// # Ошибки
    ///
    /// `None`, если номер вне диапазона или сегмент короче , чем
    /// заявлено раскладкой.
    pub fn watchdog_slot(&self, index: usize) -> Option<&WatchdogSlot> {
        if index >= watchdog::SLOTS {
            return None;
        }
        let offset = index.checked_mul(WatchdogSlot::SIZE)?;
        if offset + WatchdogSlot::SIZE > self.layout.size {
            return None;
        }
        // SAFETY: адрес лежит внутри отображённой области (проверено
        // выше), а `WatchdogSlot` - `#[repr(C)]` из 23 `AtomicU32`,
        // то есть 92 байта без выравнивания и без промежутков.
        // Выравнивание `AtomicU32` равно 4, а `mmap` возвращает
        // адрес, кратный странице, поэтому указатель выровнен.
        unsafe { self.cast_ptr::<WatchdogSlot>(offset).as_ref() }
    }

    /// Типизированный указатель на отображение со смещением.
    ///
    /// # Безопасность
    ///
    /// Вызывающий обязан гарантировать , что `offset + size_of::<T>()`
    /// не выходит за `layout.size` и что `T` совместим с раскладкой
    /// сегмента. Обе проверки - за пределами типовой системы.
    unsafe fn cast_ptr<T>(&self, offset: usize) -> *const T {
        // SAFETY: смещение проверено вызывающим , отображение валидно.
        unsafe { self.addr.add(offset).cast::<T>() }
    }

    /// Удалить именованный сегмент (`shm_unlink`).
    ///
    /// Вызывает **создатель** при штатном завершении. Само отображение
    /// при этом остаётся живым в уже работающих процессах - они
    /// продолжают видеть данные , но новые процессы сегмент уже не
    /// найдут.
    ///
    /// # Ошибки
    ///
    /// `false`, если имя не помещается в буфер , сегмента нет или
    /// `shm_unlink` вернул ошибку.
    pub fn unlink(name: SegmentName) -> bool {
        let Some(cname) = CName::new(name.0) else {
            return false;
        };
        // SAFETY: `cname` - корректная NUL-терминированная строка.
        unsafe { crate::sysv::shm_unlink(cname.as_ptr()) == 0 }
    }
}

impl Drop for SharedSegment {
    fn drop(&mut self) {
        if self.addr.is_null() {
            return;
        }
        if self.name.0.is_empty() {
            // Память из кучи , а не отображение: освобождаем правильно.
            // SAFETY: адрес получен из `alloc_zeroed` с той же
            // раскладкой в `from_raw`, других владельцев нет.
            unsafe {
                let l = std::alloc::Layout::from_size_align(
                    self.layout.size,
                    core::mem::align_of::<WatchdogSlot>(),
                );
                if let Ok(l) = l {
                    std::alloc::dealloc(self.addr, l);
                }
            }
            self.addr = core::ptr::null_mut();
            return;
        }
        // SAFETY: `addr` получен из успешного `mmap` и не освобождался.
        unsafe {
            crate::sysv::munmap(self.addr.cast(), self.layout.size);
        }
        self.addr = core::ptr::null_mut();
    }
}

/// NUL-терминированная копия имени для системных вызовов.
struct CName(heapless::Vec<u8, NAME_CAPACITY>);

impl CName {
    /// Скопировать имя и добавить завершающий ноль.
    ///
    /// Длина проверяется **до** копирования: `heapless` при
    /// переполнении тихо обрезает , и обрезанное имя указало бы на
    /// чужой объект в `/dev/shm`. Обрезанный `shm_open` - это не
    /// ошибка , а доступ к чужому сегменту.
    fn new(name: &str) -> Option<Self> {
        if name.len() + 1 > NAME_CAPACITY {
            return None;
        }
        let mut buf: heapless::Vec<u8, NAME_CAPACITY> = heapless::Vec::new();
        if buf.extend_from_slice(name.as_bytes()).is_err() {
            return None;
        }
        buf.push(0).ok()?;
        Some(Self(buf))
    }

    /// Указатель на строку.
    fn as_ptr(&self) -> *const c_char {
        self.0.as_ptr().cast::<c_char>()
    }
}

/// Свободный адрес разделяемой памяти.
///
/// Ничего не делает: адрес всегда есть , он выдаётся ядром. Функция
/// существует , чтобы у вызывающего не было соблазна писать
/// `SharedSegment::open(..., 0, ...)` - `mmap` с нулевой длиной
/// падает , и это плохо читаемый отказ.
pub const fn any_segment_addr() -> *mut c_void {
    core::ptr::null_mut()
}

/// Удобная обёртка: сегмент по раскладке.
///
/// # Ошибки
///
/// `None`, если сегмент не удалось открыть.
pub fn open_layout(layout: ShmLayout, ownership: Ownership) -> Option<SharedSegment> {
    SharedSegment::open(layout.name, layout.size, ownership)
}

/// Проверить , что раскладка пригодна для открытия.
///
/// # Ошибки
///
/// [`Error::Unsupported`] , если имя непригодно или размер нулевой.
pub fn validate_layout(layout: ShmLayout) -> Result<(), Error> {
    if layout.size == 0 || !layout.name.is_valid() {
        return Err(Error::Unsupported);
    }
    if !layout.is_compatible_with_c() {
        return Err(Error::InvalidState);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering::Relaxed;

    #[test]
    fn layout_validation() {
        assert!(ShmLayout::new(SegmentName::RADIO_STATS, 92, 4).is_some());
        assert!(ShmLayout::new(SegmentName::RADIO_STATS, 93, 4).is_none());
        assert!(ShmLayout::new(SegmentName::RADIO_STATS, 0, 4).is_none());
        assert!(ShmLayout::new(SegmentName::RADIO_STATS, 92, 0).is_none());
    }

    #[test]
    fn alignment_rounding() {
        assert_eq!(ShmLayout::aligned_size(93, 4), 96);
        assert_eq!(ShmLayout::aligned_size(92, 4), 92);
    }

    #[test]
    fn watchdog_sizes_match_c() {
        // 23 × u32 = 92 байта , как `shared_mem_process_stats` в C.
        assert_eq!(WatchdogSlot::SIZE, 92);
        assert_eq!(WatchdogSlot::FIELD_COUNT, 23);
        assert_eq!(core::mem::size_of::<WatchdogSlot>(), WatchdogSlot::SIZE);
        assert_eq!(core::mem::align_of::<WatchdogSlot>(), 4);
    }

    #[test]
    fn reserved_fields_fill_the_gap() {
        assert_eq!(
            WatchdogSlot::NAMED_FIELDS + WatchdogSlot::RESERVED_FIELDS,
            WatchdogSlot::FIELD_COUNT
        );
        assert_eq!(WatchdogSlot::RESERVED_FIELDS, 15);
    }

    #[test]
    fn watchdog_segment_layout() {
        let l = watchdog::LAYOUT;
        assert_eq!(l.size, WatchdogSlot::SIZE * watchdog::SLOTS);
        assert_eq!(l.size, 736);
        assert_eq!(l.size % DEFAULT_ALIGN, 0);
        assert!(l.verified, "размер heartbeat выводится из числа полей C");
    }

    fn slot() -> &'static WatchdogSlot {
        Box::leak(Box::new(unsafe { core::mem::zeroed() }))
    }

    #[test]
    fn watchdog_liveness() {
        let s = slot();
        s.heartbeat(UptimeMillis(1_000));
        assert!(s.is_alive(UptimeMillis(1_200), 500));
        assert!(!s.is_alive(UptimeMillis(2_000), 500));
    }

    #[test]
    fn watchdog_liveness_survives_32bit_wraparound() {
        let s = slot();
        let base = UptimeMillis(u64::from(u32::MAX) + 1);
        s.heartbeat(base);
        // Через 200 мс значение перекрутилось через ноль - процесс
        // обязан оставаться живым.
        assert!(s.is_alive(base + 200, 500));
    }

    #[test]
    fn watchdog_empty_slot_is_not_alive() {
        let s = slot();
        assert!(!s.is_alive(UptimeMillis(10_000), 500));
    }

    #[test]
    fn watchdog_reset_clears_everything() {
        let s = slot();
        s.heartbeat(UptimeMillis(5_000));
        s.mark_rx(UptimeMillis(5_000));
        s.mark_tx(UptimeMillis(5_000));
        s.count_loop(10);
        s.raise_alarm(alarm::RADIO_INTERFACE_BEHIND, UptimeMillis(5_000));
        s.reset();
        assert_eq!(s.last_active(), UptimeMillis(0));
        assert_eq!(s.loop_counter.load(Relaxed), 0);
        assert_eq!(s.alarm_flags.load(Relaxed), alarm::NONE);
        assert_eq!(s.reserved[0].load(Relaxed), 0);
    }

    #[test]
    fn loop_average_is_exponential() {
        let s = slot();
        for _ in 0..100 {
            s.count_loop(8);
        }
        let avg = s.avg_loop_time_ms.load(Relaxed);
        assert!((6..=9).contains(&avg), "среднее {avg} должно сойтись к 8");
        assert_eq!(s.max_loop_time_ms.load(Relaxed), 8);
        assert_eq!(s.loop_counter.load(Relaxed), 100);
    }

    #[test]
    fn alarms_are_bits() {
        let s = slot();
        s.raise_alarm(alarm::RADIO_INTERFACE_BEHIND, UptimeMillis(7));
        s.raise_alarm(alarm::RADIO_STREAM_RESTARTED, UptimeMillis(9));
        assert_eq!(
            s.alarm_flags.load(Relaxed),
            alarm::RADIO_INTERFACE_BEHIND | alarm::RADIO_STREAM_RESTARTED
        );
        assert_eq!(s.alarm_time.load(Relaxed), 9);
        s.clear_alarms();
        assert_eq!(s.alarm_flags.load(Relaxed), alarm::NONE);
    }

    #[test]
    fn slot_roles_map_to_binaries() {
        assert_eq!(watchdog::SlotRole::VehicleRouter.bin_name(), "b-fruby-rt-vehicle");
        assert_eq!(watchdog::SlotRole::StationRadio.bin_name(), "b-fruby-radio-station");
        assert_eq!(watchdog::SlotRole::ALL.len(), watchdog::SLOTS);
    }

    #[test]
    fn bootloader_is_not_supervised() {
        // Супервизор не может перезапустить самого себя по своему же
        // слоту: это зацикливание.
        assert!(!watchdog::SlotRole::Bootloader.is_supervised());
        for r in watchdog::SlotRole::ALL {
            if r != watchdog::SlotRole::Bootloader {
                assert!(r.is_supervised(), "{r:?} должен быть под надзором");
            }
        }
    }

    #[test]
    fn watchdog_segment_slots_are_addressable() {
        let seg = WatchdogSegment::detached();
        for i in 0..watchdog::SLOTS {
            assert!(seg.slot(i).is_some(), "слот {i} недоступен");
        }
        assert!(seg.slot(watchdog::SLOTS).is_none());
    }

    #[test]
    fn watchdog_segment_heartbeat_and_liveness() {
        let seg = WatchdogSegment::detached();
        let now = UptimeMillis(1_000);
        assert!(seg.heartbeat(watchdog::SlotRole::VehicleRouter, now));
        assert!(seg.is_alive(watchdog::SlotRole::VehicleRouter, now + 100, 500));
        // Соседний слот остался пустым и считается мёртвым.
        assert!(!seg.is_alive(watchdog::SlotRole::StationRouter, now + 100, 500));
    }

    #[test]
    fn stale_roles_skips_bootloader_and_live_ones() {
        let seg = WatchdogSegment::detached();
        let now = UptimeMillis(1_000);
        seg.heartbeat(watchdog::SlotRole::VehicleRouter, now);
        let stale = seg.stale_roles(now + 100, 500);
        assert!(stale.contains(&watchdog::SlotRole::StationRouter));
        assert!(!stale.contains(&watchdog::SlotRole::VehicleRouter));
        assert!(!stale.contains(&watchdog::SlotRole::Bootloader));
    }

    #[test]
    fn segment_names_are_valid_posix() {
        for n in [
            SegmentName::RADIO_STATS,
            SegmentName::VIDEO_STREAM_INFO,
            SegmentName::ROUTER_VEHICLES,
            SegmentName::I2C_ROTARY,
            SegmentName::WATCHDOG,
        ] {
            assert!(n.is_valid(), "{n:?} непригодно для shm_open");
            assert!(!n.base_name().is_empty());
            assert!(!n.base_name().starts_with('/'));
        }
    }

    #[test]
    fn invalid_segment_names_are_rejected() {
        assert!(!SegmentName("").is_valid());
        assert!(!SegmentName("no-slash").is_valid());
        assert!(!SegmentName("/two/parts").is_valid());
        assert!(!SegmentName("/").is_valid());
    }

    #[test]
    fn zero_sized_segment_is_rejected() {
        assert!(SharedSegment::open(SegmentName::WATCHDOG, 0, Ownership::Writer).is_none());
    }

    #[test]
    fn detached_segment_exposes_bytes() {
        // SAFETY: блок выделяется под нас и не делится с другими
        // владельцами; `SharedSegment` освободит его в `Drop`.
        let addr = unsafe {
            std::alloc::alloc_zeroed(std::alloc::Layout::from_size_align(64, 4).unwrap()).cast()
        };
        let seg = SharedSegment::from_raw(addr, 64, false);
        // SAFETY: отображение выделено нами и не делится с другими
        // владельцами.
        let b = unsafe { seg.bytes() };
        assert_eq!(b.len(), 64);
        assert!(b.iter().all(|v| *v == 0));
        assert!(!seg.was_created());
    }

    #[test]
    fn unverified_layouts_are_visible_as_such() {
        let l = ShmLayout::new(SegmentName::RADIO_STATS, 2048, 4).unwrap();
        assert!(!l.verified);
        assert!(!l.is_compatible_with_c());
        assert_eq!(validate_layout(l), Err(Error::InvalidState));

        let v = ShmLayout::verified(SegmentName::RADIO_STATS, 2048, 4).unwrap();
        assert!(v.verified);
        assert_eq!(validate_layout(v), Ok(()));
    }

    #[test]
    fn unlink_of_absent_segment_fails() {
        assert!(!SharedSegment::unlink(SegmentName("/FRUBY_TEST_NO_SUCH_SEGMENT")));
    }

    #[test]
    fn cname_fits_or_fails_cleanly() {
        assert!(CName::new("/SYSTEM_SHARED_MEM_WATCHDOG").is_some());
        assert!(CName::new("/").is_some());
        let long = format!("/{}", "X".repeat(NAME_CAPACITY + 10));
        assert!(CName::new(&long).is_none());
    }

    #[test]
    fn watchdog_empty_slot_is_not_alive_early_after_boot() {
        // Аптайм меньше порога , а слот пуст. Раньше разность
        // `now - 0` попадала в `max_stale_ms` , и процесс , который
        // ещё не стартовал , считался живым.
        let s = slot();
        assert!(!s.is_alive(UptimeMillis(300), 500));
        assert!(!s.is_alive(UptimeMillis(0), 500));
        assert!(!s.has_heartbeat());
    }

    #[test]
    fn heartbeat_at_zero_uptime_is_not_mistaken_for_empty() {
        // Первые 500 мс после загрузки аптайм равен нулю. Записать
        // ноль - значит пометить слот как «никогда не писал».
        let s = slot();
        s.heartbeat(UptimeMillis(0));
        assert!(s.has_heartbeat());
        assert!(s.is_alive(UptimeMillis(100), 500));
        assert!(!s.is_alive(UptimeMillis(1_000), 500));
    }

    #[test]
    fn reset_returns_slot_to_empty() {
        let s = slot();
        s.heartbeat(UptimeMillis(0));
        assert!(s.has_heartbeat());
        s.reset();
        assert!(!s.has_heartbeat());
        assert!(!s.is_alive(UptimeMillis(10), 500));
    }

    #[test]
    fn slot_empty_mark_is_zero() {
        assert_eq!(SLOT_EMPTY_MARK, 0);
    }

    #[test]
    fn open_read_retry_gives_up_after_attempts() {
        // Сегмента нет и не появится: после всех попыток должен
        // вернуться `None`, а не висеть вечно.
        assert!(SharedSegment::open_read_retry(
            SegmentName("/FRUBY_TEST_NEVER_APPEARS"),
            64,
            3,
            0,
        )
        .is_none());
    }

    #[test]
    fn open_read_retry_treats_zero_attempts_as_one() {
        // Ноль попыток - опечатка, а не «не пробать вовсе».
        let _lock = ipc_guard();
        let _ = SharedSegment::unlink(SegmentName::WATCHDOG);
        assert!(SharedSegment::open_read_retry(SegmentName::WATCHDOG, watchdog::LAYOUT.size, 0, 0)
            .is_none());
    }

    /// Сериализовать тесты, работающие с настоящими сегментами.
    fn ipc_guard() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_segment_is_created_zeroed_once_and_unlinked() {
        let _lock = ipc_guard();
        let _ = SharedSegment::unlink(SegmentName::WATCHDOG);
        let size = watchdog::LAYOUT.size;

        // Первый писатель создаёт и обнуляет.
        let creator = SharedSegment::open_write(SegmentName::WATCHDOG, size).expect("создать");
        assert!(creator.was_created());
        assert_eq!(creator.layout().size, size);

        // Второй писатель присоединяется и НЕ обнуляет: иначе
        // перезапуск любого процесса стирал бы состояние остальных.
        let second = SharedSegment::open_write(SegmentName::WATCHDOG, size).expect("присоединиться");
        assert!(!second.was_created());

        // Читатель видит полную длину и тоже не обнуляет.
        let reader = SharedSegment::open_read(SegmentName::WATCHDOG, size).expect("прочитать");
        assert!(!reader.was_created());

        // Пишем через одного писателя, читаем через читателя.
        {
            let slot = second.watchdog_slot(0).expect("слот 0");
            slot.heartbeat(UptimeMillis(1_234));
        }
        let seen = reader.watchdog_slot(0).expect("слот 0").last_active();
        assert_eq!(seen, UptimeMillis(1_234), "разделяемая память не разделяется");

        drop(creator);
        drop(second);
        drop(reader);
        assert!(SharedSegment::unlink(SegmentName::WATCHDOG));
        assert!(SharedSegment::open_read(SegmentName::WATCHDOG, size).is_none());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_reader_rejects_segment_shorter_than_layout() {
        let _lock = ipc_guard();
        let _ = SharedSegment::unlink(SegmentName::WATCHDOG);
        // Писатель создаёт сегмент меньше раскладки.
        let writer = SharedSegment::open_write(SegmentName::WATCHDOG, 64).expect("создать");
        assert!(writer.was_created());
        // Читатель по раскладке обязан отказаться, а не читать за границу.
        assert!(SharedSegment::open_read(SegmentName::WATCHDOG, watchdog::LAYOUT.size).is_none());
        drop(writer);
        let _ = SharedSegment::unlink(SegmentName::WATCHDOG);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_reader_retry_waits_for_writer() {
        let _lock = ipc_guard();
        let _ = SharedSegment::unlink(SegmentName::WATCHDOG);
        let size = watchdog::LAYOUT.size;

        // Читатель пробует до появления сегмента.
        let h = std::thread::spawn(move || {
            SharedSegment::open_read_retry(SegmentName::WATCHDOG, size, 50, 10)
        });
        std::thread::sleep(std::time::Duration::from_millis(80));
        let writer = SharedSegment::open_write(SegmentName::WATCHDOG, size).expect("создать");
        let seg = h.join().expect("поток читателя").expect("читатель дождался");
        assert_eq!(seg.layout().size, size);
        drop(writer);
        drop(seg);
        let _ = SharedSegment::unlink(SegmentName::WATCHDOG);
    }
}
