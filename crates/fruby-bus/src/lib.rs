//! FabricRuby/crates/fruby-bus/src/lib.rs
//! 
//! fruby-bus - шина межпроцессного обмена FabricRuby.
//! Соединяет восемь бинарников: семь runtime-процессов
//! (включая `b-fruby-tools`) и супервизор `b-fruby-bootloader`.
//!
//! ## Четыре механизма , каждый под свою задачу.
//!
//! | Механизм | Что перевозит | Модуль |
//! |---|---|---|
//! | очереди сообщений SysV | короткие сообщения , адресуемые по номеру канала | [`msgqueue`] |
//! | разделяемая память POSIX | крупные структуры состояния и видеопоток | [`shm`] |
//! | именованные семафоры | порядок доступа к общей памяти | [`sem`] |
//! | слоты heartbeat в shm | признак «я жив» для супервизора | [`shm::WatchdogSegment`] |
//! 
//! Плюс набор каналов ([`ChannelId`]) и формат сообщения
//! ([`msgqueue::busMessage`]), которые определяют , кто кому что шлёт.
//!
//! ## Кто с кем говорит
//!
//! Все процессы запущены `b-fruby-bootloader` параллельно и **не**
//! вызывают друг друга. Общаются только через эту шину.
//!
//! ```text
//! b-fruby-rt-vehicle    -> b-fruby-radio-vehicle  видео в эфир      (shm)
//! b-fruby-radio-vehicle -> b-fruby-rt-vehicle     принятые RC       (msgqueue)
//! b-fruby-rt-station    -> b-fruby-central        видео для показа  (shm)
//! b-fruby-central       -> b-fruby-rt-station     команды юзера     (msgqueue)
//! b-fruby-central       -> b-fruby-mavlink        запросы телеметрии(msgqueue)
//! b-fruby-mavlink       -> b-fruby-rt-station     телеметрия в эфир (msgqueue)
//! b-fruby-mavlink       -> b-fruby-central        телеметрия на OSD (msgqueue)
//! b-fruby-radio-station -> b-fruby-rt-station     RSSI, статистика  (shm)
//! все                   -> b-fruby-tools          логи, alive       (msgqueue)
//! все                   -> свой слот heartbeat    (shm, запись)
//! b-fruby-bootloader    <- слоты heartbeat всех   (shm, опрос)
//! ```
//!
//! ## Чего крейт не делает
//!
//! Не работает по сети - всё локально , в пределах одной машины.
//! Не читает файлы (это `fruby-hal::Storage`). Не разбирает пакеты
//! (это `fruby-protocol`). Не шифрует (это `fruby-crypto`). Не рисует
//! (это `fruby-render` и `fruby-ui`). Не управляет процессами (это
//! `fruby-hal::System` и загрузчик). Не работает с радио (это
//! `fruby-hal::RadioInterface`).
//!
//! `fruby-bus` - только транспорт между процессами: почтальон , а не
//! фабрика и не склад.
//!
//! ## Что исправлено относительно C
//!
//! 1. **Ключ очереди не зависит от файла.** В C он получался через
//!    `ftok(path, proj_id)`, то есть от inode. Пересоздали файл - сменился
//!    inode , сменился ключ , и процессы «потеряли» очереди друг друга.
//!    Здесь ключ считается от фиксированной константы и номера канала:
//!    [`msgqueue::ChannelKey::for_channel`].
//! 2. **`msgrcv` больше не затирает сообщение.** Буфер очереди
//!    начинается с `long mtype`, который пишет ядро. Раньше указатель
//!    передавался сразу на данные , и ядро затирало первые восемь байт -
//!    CRC , идентификатор и длину. Приём не работал.
//! 3. **Сегмент не обнуляется при каждом открытии на запись.** В C
//!    `open_shared_mem(..., readOnly=false)` затирал весь сегмент , и
//!    перезапуск любого процесса уничтожал статистику остальных.
//!    Здесь обнуление только при создании.
//! 4. **Ресурсы удаляются явно.** Очереди , сегменты и семафоры
//!    снимаются создателем при штатном завершении. В C `shm_unlink`
//!    и `sem_unlink` были закомментированы почти везде , и имена
//!    «протекали» в `/dev/shm` между перезагрузками.
//! 5. **Семафор хранит настоящий `sem_t*`.** Тип `c_int` обрезал
//!    указатель до 32 бит , а станция Radxa 3W - 64-битная.
//! 6. **Значения каналов сохранены дословно** (68 , 69 , 70 , 71 , 77,
//!    78 , 81 , 83 , 84 и `PROJ_ID = 117`). Они попадают в заголовок
//!    сообщения и в лог , и на них рассчитывают внешние утилиты вроде
//!    `ruby_logger`. Менять их нельзя.
//!
//! ## Границы
//!
//! Крейт - слой 2 (абстракции ОС). Он зависит от `fruby-types`,
//! `fruby-crypto` и `heapless`, но не знает ни о ролях , ни о
//! бинарниках: роли подставляют снаружи через [`ChannelFor`] и
//! `fruby_daemon_common::ipc_daemon::channels_for`.

#![warn(missing_docs)]
// Документация небезопасных функций в этом крейте написана по-русски:
// заголовок раздела «Безопасность» вместо `# Safety`. Линт clippy ищет
// именно английский заголовок и без пользы ругается на каждую функцию.
// Требование проекта - русский текст во всех комментариях, поэтому
// проверка отключена явно, а не молча.
#![allow(clippy::missing_safety_doc)]

use core::fmt;

// Типы из `fruby-types` , которые входят в сигнатуры этого крейте,
// пере-экспортируются: пользователь не должен подключать
// `fruby-types` только ради `WatchdogSegment::heartbeat` или ради
// оператора `?` над возвратом `Result` этого крейте.
pub use fruby_types::{ComponentId, Error, UptimeMillis};

pub mod msgqueue;
pub mod sem;
pub mod shm;
pub mod sysv;

pub use msgqueue::{
    BUS_KEY_PATH, ChannelKey, ChannelStats, MsgQueue, OpenPolicy, PROJ_ID, QueueStat, Router,
    busMessage,
};
pub use msgqueue::PAYLOAD_CAPACITY;
pub use sem::{NamedSemaphore, SemOpen, SemaphoreName};
pub use shm::{
    Ownership, SegmentName, SharedSegment, ShmLayout, WatchdogSegment, WatchdogSlot, alarm,
    layouts, watchdog,
};

/// Руководство для программиста проверяется как doctest'ы.
///
/// Объявление существует только при сборе документации: во время
/// обычной сборки крейта оно не компилируется. Благодаря ему каждый
/// пример кода из `USAGE.md` проверяется компилятором, и пример ,
/// который перестал соответствовать API , ломает `cargo test`,
/// а не тихо расходится с кодом.
#[cfg(doctest)]
#[doc = include_str!("../USAGE.md")]
struct UsageExamples;

/// Канал bus (`type_ruby_bus_channels_type`).
///
/// Идентификаторы совпадают с `bus_CHANNEL_TYPE_*` из `ruby_bus.h`,
/// потому что они записываются в заголовок сообщения и в логи.
///
/// Нумерация разреженная и не подряд: так в C , и менять её нельзя.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum ChannelId {
    /// Маршрутизатор → RC.
    RouterToRc = 68,
    /// RC → маршрутизатор.
    RcToRouter = 69,
    /// Маршрутизатор → UI.
    RouterToCentral = 70,
    /// UI → маршрутизатор.
    CentralToRouter = 71,
    /// Маршрутизатор → команды.
    RouterToCommands = 77,
    /// Команды → маршрутизатор.
    CommandsToRouter = 78,
    /// Команды камеры/видео.
    CsiVideoCommands = 81,
    /// Маршрутизатор → телеметрия.
    RouterToTelemetry = 83,
    /// Телеметрия → маршрутизатор.
    TelemetryToRouter = 84,
}

impl ChannelId {
    /// Числовой код.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Разбор кода.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            68 => Some(Self::RouterToRc),
            69 => Some(Self::RcToRouter),
            70 => Some(Self::RouterToCentral),
            71 => Some(Self::CentralToRouter),
            77 => Some(Self::RouterToCommands),
            78 => Some(Self::CommandsToRouter),
            81 => Some(Self::CsiVideoCommands),
            83 => Some(Self::RouterToTelemetry),
            84 => Some(Self::TelemetryToRouter),
            _ => None,
        }
    }

    /// Все каналы , которые должен открыть маршрутизатор.
    ///
    /// Семь каналов - по числу участников маршрутизации.
    pub const ROUTER_CHANNELS: [ChannelId; 7] = [
        Self::RouterToRc,
        Self::RouterToCentral,
        Self::RouterToTelemetry,
        Self::RouterToCommands,
        Self::RcToRouter,
        Self::TelemetryToRouter,
        Self::CommandsToRouter,
    ];

    /// Все известные каналы , включая видео.
    ///
    /// Маршрутизатор открывает не все: видео в SysV-очередь не идёт,
    /// оно передаётся через разделяемую память. Но [`ChannelId::from_code`]
    /// обязан уметь разобрать и его - код попадает в лог.
    pub const ALL: [ChannelId; 9] = [
        Self::RouterToRc,
        Self::RcToRouter,
        Self::RouterToCentral,
        Self::CentralToRouter,
        Self::RouterToCommands,
        Self::CommandsToRouter,
        Self::CsiVideoCommands,
        Self::RouterToTelemetry,
        Self::TelemetryToRouter,
    ];

    /// Имя канала для журнала и `--help`.
    ///
    /// Пишется в лог рядом с числовым кодом: по одному коду в
    /// отчёте не разберёшься , а по одному имени - потеряешь
    /// сопоставление с C.
    pub const fn name(self) -> &'static str {
        match self {
            Self::RouterToRc => "router_to_rc",
            Self::RcToRouter => "rc_to_router",
            Self::RouterToCentral => "router_to_central",
            Self::CentralToRouter => "central_to_router",
            Self::RouterToCommands => "router_to_commands",
            Self::CommandsToRouter => "commands_to_router",
            Self::CsiVideoCommands => "csi_video_commands",
            Self::RouterToTelemetry => "router_to_telemetry",
            Self::TelemetryToRouter => "telemetry_to_router",
        }
    }

    /// `true`, если канал входит в набор маршрутизатора.
    pub const fn is_router_channel(self) -> bool {
        Self::contains(&Self::ROUTER_CHANNELS, self)
    }

    /// `true`, если `needle` есть в `haystack`.
    ///
    /// Своя реализация вместо `contains`, потому что на срезе
    /// констант из C в std нет подходящего `const`-метода.
    pub const fn contains(haystack: &[Self], needle: Self) -> bool {
        let mut i = 0;
        while i < haystack.len() {
            if same(haystack[i], needle) {
                return true;
            }
            i += 1;
        }
        false
    }
}

/// Сравнение вариантов перечисления в константном контексте.
///
/// На срезе констант из C единственный доступный путь - сравнение
/// целочисленных дискриминантов.
const fn same(a: ChannelId, b: ChannelId) -> bool {
    a as u8 == b as u8
}

impl fmt::Display for ChannelId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ({})", self.name(), self.code())
    }
}

/// Максимальный размер сообщения (`bus_CHANNEL_MAX_MSG_SIZE`).
///
/// Ровно столько приходит в C в `struct { long type; char data[1600]; }`.
pub const MAX_MSG_SIZE: usize = fruby_types::MAX_IPC_MSG_SIZE;

/// Канал ожидания: максимум 16 одновременных (`MAX_CHANNELS`).
pub const MAX_CHANNELS: usize = 16;

/// Приоритет сообщения SysV (`msg.type`).
///
/// В C всегда 1; разделение на «служебные» и «данные» сообщения
/// введено , чтобы логгер не принимал пакеты (см. `ruby_logger.cpp`,
/// где тип 1 - системный лог , а всё остальное отбрасывается).
///
/// Тип - `c_long`, а не `i64`: на станции Radxa 3W и на борту
/// SSC338Q `long` разной ширины , а поле `mtype` в очереди имеет
/// ровно ту ширину , что `long` на целевой платформе.
pub const MSG_TYPE_DATA: core::ffi::c_long = 1;

/// Системное сообщение (журнал).
pub const MSG_TYPE_LOG: core::ffi::c_long = 2;

/// Канал , привязанный к компоненту пакета и направлению.
///
/// Упрощает маршрутизацию: вместо ручного выбора bus-канала в каждом
/// месте вызывающий код передаёт [`ComponentId`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelFor {
    /// Маршрутизатор → UI.
    RouterToCentral,
    /// UI → маршрутизатор.
    CentralToRouter,
    /// Маршрутизатор → телеметрия.
    RouterToTelemetry,
    /// Телеметрия → маршрутизатор.
    TelemetryToRouter,
    /// Маршрутизатор → RC.
    RouterToRc,
    /// RC → маршрутизатор.
    RcToRouter,
    /// Маршрутизатор → команды.
    RouterToCommands,
    /// Команды → маршрутизатор.
    CommandsToRouter,
}

impl ChannelFor {
    /// Идентификатор канала.
    pub const fn id(self) -> ChannelId {
        match self {
            Self::RouterToCentral => ChannelId::RouterToCentral,
            Self::CentralToRouter => ChannelId::CentralToRouter,
            Self::RouterToTelemetry => ChannelId::RouterToTelemetry,
            Self::TelemetryToRouter => ChannelId::TelemetryToRouter,
            Self::RouterToRc => ChannelId::RouterToRc,
            Self::RcToRouter => ChannelId::RcToRouter,
            Self::RouterToCommands => ChannelId::RouterToCommands,
            Self::CommandsToRouter => ChannelId::CommandsToRouter,
        }
    }

    /// Канал , соответствующий компоненту пакета и направлению.
    ///
    /// `from_router` задаёт направление: `true` - сообщение от
    /// маршрутизатора к компоненту , `false` - обратно.
    ///
    /// `None` для компонентов без очереди: видео идёт через
    /// разделяемую память , локальное управление не выходит из
    /// процесса , а служебные сообщения Ruby разбирает сам
    /// маршрутизатор.
    pub const fn for_component(from_router: bool, component: ComponentId) -> Option<Self> {
        if from_router {
            match component {
                ComponentId::Rc => Some(Self::RouterToRc),
                ComponentId::Telemetry => Some(Self::RouterToTelemetry),
                ComponentId::Commands => Some(Self::RouterToCommands),
                _ => None,
            }
        } else {
            match component {
                ComponentId::Rc => Some(Self::RcToRouter),
                ComponentId::Telemetry => Some(Self::TelemetryToRouter),
                ComponentId::Commands => Some(Self::CommandsToRouter),
                _ => None,
            }
        }
    }

    /// `true`, если сообщение идёт от маршрутизатора.
    pub const fn is_from_router(self) -> bool {
        matches!(
            self,
            Self::RouterToCentral
                | Self::RouterToTelemetry
                | Self::RouterToRc
                | Self::RouterToCommands
        )
    }
}

/// Время последней активности канала.
pub type ChannelLastActive = fruby_types::UptimeMillis;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_codes_roundtrip() {
        for id in ChannelId::ALL {
            assert_eq!(ChannelId::from_code(id.code()), Some(id));
        }
        // Коды 72..76 и 79..80 в C не используются.
        for code in [0u8, 1, 67, 72, 76, 79, 82, 85, 255] {
            assert_eq!(ChannelId::from_code(code), None, "код {code} не должен разбираться");
        }
    }

    #[test]
    fn channel_codes_match_c() {
        // Значения из `ruby_bus.h` менять нельзя: см. ASSUMPTIONS.md,
        // Q-5.
        assert_eq!(ChannelId::RouterToRc.code(), 68);
        assert_eq!(ChannelId::RcToRouter.code(), 69);
        assert_eq!(ChannelId::RouterToCentral.code(), 70);
        assert_eq!(ChannelId::CentralToRouter.code(), 71);
        assert_eq!(ChannelId::RouterToCommands.code(), 77);
        assert_eq!(ChannelId::CommandsToRouter.code(), 78);
        assert_eq!(ChannelId::CsiVideoCommands.code(), 81);
        assert_eq!(ChannelId::RouterToTelemetry.code(), 83);
        assert_eq!(ChannelId::TelemetryToRouter.code(), 84);
    }

    #[test]
    fn router_channel_set() {
        assert_eq!(ChannelId::ROUTER_CHANNELS.len(), 7);
        for ch in ChannelId::ROUTER_CHANNELS {
            assert!(ch.is_router_channel(), "{ch:?} должен быть в наборе");
        }
        assert!(!ChannelId::CsiVideoCommands.is_router_channel());
    }

    #[test]
    fn all_covers_router_channels_and_more() {
        for ch in ChannelId::ROUTER_CHANNELS {
            assert!(ChannelId::contains(&ChannelId::ALL, ch));
        }
        assert_eq!(ChannelId::ALL.len(), 9);
    }

    #[test]
    fn display_carries_name_and_code() {
        // В лог попадают оба: имя читаемо , код сверяется с C.
        let s = format!("{}", ChannelId::RouterToTelemetry);
        assert!(s.contains("router_to_telemetry"));
        assert!(s.contains('8') || s.contains("83"));
    }

    #[test]
    fn channel_names_are_unique() {
        for i in 0..ChannelId::ALL.len() {
            for j in (i + 1)..ChannelId::ALL.len() {
                assert_ne!(
                    ChannelId::ALL[i].name(),
                    ChannelId::ALL[j].name(),
                    "имена каналов должны различаться"
                );
            }
        }
    }

    #[test]
    fn component_routing() {
        let c = ChannelFor::for_component(true, ComponentId::Rc);
        assert_eq!(c, Some(ChannelFor::RouterToRc));
        let c = ChannelFor::for_component(false, ComponentId::Telemetry);
        assert_eq!(c, Some(ChannelFor::TelemetryToRouter));
        // Видео идёт напрямую в UI , а не через отдельный канал.
        assert_eq!(ChannelFor::for_component(true, ComponentId::Video), None);
        assert_eq!(ChannelFor::for_component(false, ComponentId::Video), None);
    }

    #[test]
    fn components_without_channel_are_rejected() {
        for comp in [
            ComponentId::LocalControl,
            ComponentId::Video,
            ComponentId::Ruby,
            ComponentId::Audio,
            ComponentId::Reserved,
        ] {
            assert_eq!(ChannelFor::for_component(true, comp), None);
            assert_eq!(ChannelFor::for_component(false, comp), None);
        }
    }

    #[test]
    fn direction_is_reflected_in_channel_for() {
        assert!(ChannelFor::RouterToRc.is_from_router());
        assert!(!ChannelFor::RcToRouter.is_from_router());
        for cf in [
            ChannelFor::RouterToCentral,
            ChannelFor::RouterToTelemetry,
            ChannelFor::RouterToCommands,
        ] {
            assert!(cf.is_from_router());
        }
    }

    #[test]
    fn channel_for_ids_are_distinct() {
        let all = [
            ChannelFor::RouterToCentral,
            ChannelFor::CentralToRouter,
            ChannelFor::RouterToTelemetry,
            ChannelFor::TelemetryToRouter,
            ChannelFor::RouterToRc,
            ChannelFor::RcToRouter,
            ChannelFor::RouterToCommands,
            ChannelFor::CommandsToRouter,
        ];
        for i in 0..all.len() {
            for j in (i + 1)..all.len() {
                assert_ne!(all[i].id(), all[j].id());
            }
        }
    }

    #[test]
    fn message_sizes_match_c() {
        // 1600 байт данных в C-структуре сообщения.
        assert_eq!(MAX_MSG_SIZE, 1600);
        assert_eq!(MAX_CHANNELS, 16);
        assert_eq!(MSG_TYPE_DATA, 1);
        assert_eq!(MSG_TYPE_LOG, 2);
    }

    #[test]
    fn every_channel_has_a_unique_key() {
        // Ключ строится из номера канала, поэтому совпадение двух
        // ключей означало бы коллизию, при которой процессы
        // перехватывали бы чужие сообщения.
        let mut keys: heapless::Vec<ChannelKey, 16> = heapless::Vec::new();
        for ch in ChannelId::ALL {
            keys.push(ChannelKey::for_channel(ch)).unwrap();
        }
        for i in 0..keys.len() {
            for j in (i + 1)..keys.len() {
                assert_ne!(keys[i], keys[j], "каналы {i} и {j} дали один ключ");
            }
        }
    }
}
