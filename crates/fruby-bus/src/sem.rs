//! FabricRuby/crates/fruby-bus/src/sem.rs
//!
//! Именованные семафоры - синхронизация между процессами.
//!
//! Источник: `code/base/config_obj_names.h` (22 имени) и вызовы
//! `is_semaphore_signaled_clear()` в `code/base/base.cpp`.
//!
//! ## Зачем они рядом с разделяемой памятью
//!
//! Разделяемая память даёт общие **данные** , но не даёт
//! **упорядоченности**. Два процесса , пишущие в один сегмент , должны
//! договориться , кто первый. В C это делалось вызовами `sem_wait`
//! и `sem_post` внутри `shared_mem.c`; здесь тот же набор имён и тот
//! же протокол.
//!
//! ## Список семафоров
//!
//! | Имя | Назначение |
//! |---|---|
//! | `RUBY_SEM_RESTART_VIDEO_STREAMER` | перезапуск захвата видео |
//! | `RUBY_SEM_VIDEO_STREAMER_OVERLOAD` | перегрузка видеопотока |
//! | `RUBY_SEM_RX_RADIO_HIGH_PRIORITY` | приоритетный приём |
//! | `RUBY_SEM_RX_RADIO_REG_PRIORITY` | обычный приём |
//! | `RUBY_SEM_SM_VIDEO_DATA_AVAILABLE` | данные в разделяемой памяти |
//! | `RUBY_SEM_RESTART_VEHICLE_PROCS` | команда перезапуска |
//! | `RUBY_SEM_STOP_VEHICLE_{ROUTER,TELEM_TX,COMMANDS_RX,RC_RX}` | остановка процесса |
//! | `RUBY_SEM_SET_RADIO_TX_POWER` | передача команды мощности |
//! | `RUBY_SEM_SET_MAJESTIC_PARAMS_ASYNC` | асинхронные параметры камеры |
//! | `RUBY_SEM_WD_CTRL_{PAUSE,RESUME,RESTART}` | сторожевой таймер |
//! | `RUBY_SEM_VIDEO_FILE_PLAYBACK_*` | воспроизведение записи |
//! | `RUBY_STARTED_SEMAPHORE` | защита от повторного запуска |
//!
//! Имена взяты дословно из C: они видны в `ipcs -s` на живой
//! системе , и внешние утилиты на них рассчитывают. Имя **обязано**
//! начинаться с `/` - этого требует POSIX.
//!
//! ## Отличие от C
//!
//! `sem_unlink` в C был закомментирован почти везде , и имена
//! «протекали» между перезагрузками: после `reboot` семафор оставался
//! в состоянии , которое выставил прошлый запуск. Здесь удаление
//! явное - [`NamedSemaphore::unlink`] , и делает его создатель.

use core::ffi::c_void;

use crate::sysv::{IPC_PERM_0666, O_CREAT, O_EXCL, SEM_FAILED};

/// Ёмкость имени семафора вместе с завершающим нулём.
const NAME_CAPACITY: usize = 64;

/// Начальное значение счётчика для защиты от повторного запуска.
///
/// Ноль , а не единица: `RUBY_STARTED_SEMAPHORE` открывается со
/// значением `0` , и повторный запуск системы блокируется на
/// `sem_wait` - процесс-инициатор поднимает семафор и тем самым
/// запрещает себе же повторный вход. Единица наоборот разрешала бы
/// первый же `sem_trywait` и ломала бы защиту.
pub const STARTED_SEM_INITIAL: u32 = 0;

/// Имя семафора.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SemaphoreName(pub &'static str);

impl SemaphoreName {
    /// Перезапуск видеозахвата.
    pub const RESTART_VIDEO_STREAMER: Self = Self("/RUBY_SEM_RESTART_VIDEO_STREAMER");
    /// Перегрузка видеопотока.
    pub const VIDEO_STREAMER_OVERLOAD: Self = Self("/RUBY_SEM_VIDEO_STREAMER_OVERLOAD");
    /// Семафор приоритетного приёма.
    pub const RX_RADIO_HIGH_PRIORITY: Self = Self("/RUBY_SEM_RX_RADIO_HIGH_PRIORITY");
    /// Семафор обычного приёма.
    pub const RX_RADIO_REG_PRIORITY: Self = Self("/RUBY_SEM_RX_RADIO_REG_PRIORITY");
    /// Данные видео доступны в разделяемой памяти.
    pub const SM_VIDEO_DATA_AVAILABLE: Self = Self("/RUBY_SEM_SM_VIDEO_DATA_AVAILABLE");
    /// Перезапуск процессов борта.
    pub const RESTART_VEHICLE_PROCS: Self = Self("/RUBY_SEM_RESTART_VEHICLE_PROCS");
    /// Остановка маршрутизатора борта.
    pub const STOP_VEHICLE_ROUTER: Self = Self("/RUBY_SEM_STOP_VEHICLE_ROUTER");
    /// Остановка отправки телеметрии.
    pub const STOP_VEHICLE_TELEM_TX: Self = Self("/RUBY_SEM_STOP_VEHICLE_TELEM_TX");
    /// Остановка приёма команд.
    pub const STOP_VEHICLE_COMMANDS_RX: Self = Self("/RUBY_SEM_STOP_VEHICLE_COMMANDS_RX");
    /// Остановка приёма RC.
    pub const STOP_VEHICLE_RC_RX: Self = Self("/RUBY_SEM_STOP_VEHICLE_RC_RX");
    /// Команда установки мощности.
    pub const SET_RADIO_TX_POWER: Self = Self("/RUBY_SEM_SET_RADIO_TX_POWER");
    /// Асинхронные параметры камеры Majestic.
    pub const SET_MAJESTIC_PARAMS_ASYNC: Self = Self("/RUBY_SEM_SET_MAJESTIC_PARAMS_ASYNC");
    /// Сторож приостановлен.
    pub const WD_CTRL_PAUSE: Self = Self("/RUBY_SEM_WD_CTRL_PAUSE");
    /// Сторож возобновлён.
    pub const WD_CTRL_RESUME: Self = Self("/RUBY_SEM_WD_CTRL_RESUME");
    /// Сторож: перезапуск UI.
    pub const WD_CTRL_RESTART: Self = Self("/RUBY_SEM_WD_CTRL_RESTART");
    /// Воспроизведение файла будет завершено.
    pub const VIDEO_FILE_PLAYBACK_WILL_FINISH: Self = Self("/RUBY_SEM_VIDEO_FILE_PLAYBACK_WILL_FINISH");
    /// Воспроизведение файла завершено.
    pub const VIDEO_FILE_PLAYBACK_FINISHED: Self = Self("/RUBY_SEM_VIDEO_FILE_PLAYBACK_FINISHED");
    /// Система запущена (защита от повторного запуска).
    pub const STARTED: Self = Self("/RUBY_STARTED_SEMAPHORE");

    /// `true`, если имя пригодно для `sem_open`.
    ///
    /// POSIX требует ведущий слэш и запрещает внутренние: имя из двух
    /// и более компонент ядро отвергает.
    pub fn is_valid(&self) -> bool {
        let n = self.0;
        n.starts_with('/') && !n[1..].contains('/') && n.len() > 1
    }

    /// Имя без начального слэша.
    pub fn base_name(&self) -> &'static str {
        self.0.trim_start_matches('/')
    }
}

/// Политика открытия семафора.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SemOpen {
    /// Создать , если нет; если уже создан - присоединиться.
    #[default]
    CreateOrAttach,
    /// Только присоединиться , ошибка если семафора нет.
    AttachOnly,
}

/// Именованный семафор POSIX.
///
/// Обёртка над `sem_open`/`sem_wait`/`sem_post`. Хранит настоящий
/// `sem_t*`: тип `c_int`, который стоял здесь раньше , обрезал
/// указатель до 32 бит и на aarch64 (Radxa 3W - целевая станция)
/// обращался бы по вырезанному адресу.
#[derive(Debug)]
pub struct NamedSemaphore {
    name: SemaphoreName,
    addr: *mut c_void,
}

// SAFETY: `sem_t*` POSIX-семафора можно использовать из нескольких
// потоков и процессов; синхронизация обеспечивается самим семафором.
unsafe impl Send for NamedSemaphore {}
// SAFETY: см. выше. Все методы , кроме `unlink`, только читают и меняют
// счётчик через атомарные операции ядра.
unsafe impl Sync for NamedSemaphore {}

impl NamedSemaphore {
    /// Открыть существующий семафор.
    ///
    /// Возвращает `None`, если семафор не создан.
    ///
    /// # Ошибки
    ///
    /// `None`, если имя непригодно , `sem_open` завершился ошибкой
    /// или имя не помещается в буфер.
    pub fn open_existing(name: SemaphoreName) -> Option<Self> {
        Self::open(name, SemOpen::AttachOnly, 0)
    }

    /// Открыть или создать семафор.
    ///
    /// # Ошибки
    ///
    /// `None`, если имя непригодно или `sem_open` завершился ошибкой.
    pub fn open(name: SemaphoreName, policy: SemOpen, initial: u32) -> Option<Self> {
        if !name.is_valid() {
            return None;
        }
        let cname = CName::new(name.0)?;

        // SAFETY: `cname` - корректная NUL-терминированная строка.
        // `sem_open` - вариативная функция; при `O_CREAT` передаются
        // ровно два дополнительных аргумента: `mode` и `value`.
        unsafe {
            let addr = match policy {
                SemOpen::AttachOnly => crate::sysv::sem_open_ex(cname.as_ptr(), 0, IPC_PERM_0666 as u32, 0),
                SemOpen::CreateOrAttach => {
                    let created = crate::sysv::sem_open_ex(
                        cname.as_ptr(),
                        O_CREAT | O_EXCL,
                        IPC_PERM_0666 as u32,
                        initial,
                    );
                    if created != SEM_FAILED {
                        created
                    } else {
                        // Уже существует - это штатная гонка при
                        // одновременном старте процессов.
                        crate::sysv::sem_open_ex(cname.as_ptr(), 0, IPC_PERM_0666 as u32, 0)
                    }
                }
            };
            if addr == SEM_FAILED {
                return None;
            }
            Some(Self { name, addr })
        }
    }

    /// Имя семафора.
    pub const fn name(&self) -> SemaphoreName {
        self.name
    }

    /// `true`, если семафор открыт.
    pub const fn is_open(&self) -> bool {
        !self.addr.is_null()
    }

    /// Проверить и снять сигнал без блокировки.
    ///
    /// Соответствует `is_semaphore_signaled_clear()`: возвращает
    /// `true`, если семафор был поднят и сброшен. `false` означает
    /// либо «сигнала нет» , либо «семафор не открыт» - в цикле
    /// опроса это одно и то же.
    pub fn try_signal_clear(&self) -> bool {
        if !self.is_open() {
            return false;
        }
        // SAFETY: `addr` получен из успешного `sem_open` и жив , пока
        // жив `self`.
        unsafe { crate::sysv::sem_trywait(self.addr) == 0 }
    }

    /// Ждать сигнал (блокирующий вызов).
    ///
    /// В циклах FabricRuby не применяется: горячий цикл не должен
    /// вставать. Метод нужен только в точках , где процесс уже решил
    /// ждать , - например , при остановке.
    ///
    /// # Ошибки
    ///
    /// `false`, если семафор не открыт или `sem_wait` прерван
    /// сигналом.
    pub fn wait(&self) -> bool {
        if !self.is_open() {
            return false;
        }
        // SAFETY: `addr` получен из успешного `sem_open`.
        unsafe { crate::sysv::sem_wait(self.addr) == 0 }
    }

    /// Поднять семафор (`sem_post`).
    ///
    /// # Ошибки
    ///
    /// `false`, если семафор не открыт или счётчик переполнился.
    pub fn signal(&self) -> bool {
        if !self.is_open() {
            return false;
        }
        // SAFETY: `addr` получен из успешного `sem_open`.
        unsafe { crate::sysv::sem_post(self.addr) == 0 }
    }

    /// Текущее значение счётчика (`sem_getvalue`).
    ///
    /// `-1`, если семафор не открыт или запрос не удался.
    pub fn value(&self) -> i32 {
        if !self.is_open() {
            return -1;
        }
        let mut v: i32 = 0;
        // SAFETY: `v` - корректный указатель на `int`; `addr` - из
        // успешного `sem_open`.
        unsafe {
            if crate::sysv::sem_getvalue(self.addr, &mut v as *mut i32) == 0 {
                v
            } else {
                -1
            }
        }
    }

    /// Удалить именованный семафор (`sem_unlink`).
    ///
    /// Вызывает **создатель** при штатном завершении: имя пропадает,
    /// но уже работающие процессы держат свои отображения и продолжают
    /// пользоваться семафором.
    ///
    /// # Ошибки
    ///
    /// `false`, если имя не помещается в буфер , семафора нет или
    /// `sem_unlink` вернул ошибку.
    pub fn unlink(name: SemaphoreName) -> bool {
        let Some(cname) = CName::new(name.0) else {
            return false;
        };
        // SAFETY: `cname` - корректная NUL-терминированная строка.
        unsafe { crate::sysv::sem_unlink(cname.as_ptr()) == 0 }
    }
}

impl Drop for NamedSemaphore {
    fn drop(&mut self) {
        if self.is_open() {
            // SAFETY: `addr` получен из успешного `sem_open` и не
            // освобождался.
            unsafe {
                crate::sysv::sem_close(self.addr);
            }
            self.addr = core::ptr::null_mut();
        }
    }
}

/// NUL-терминированная копия имени для системных вызовов.
struct CName(heapless::Vec<u8, NAME_CAPACITY>);

impl CName {
    /// Скопировать имя и добавить завержающий ноль.
    ///
    /// Длина проверяется **до** копирования: `heapless` при
    /// переполнении тихо обрезает , и обрезанное имя указало бы на
    /// чужой семафор. Обрезанный `sem_open` - это не ошибка , а
    /// блокировка на чужом объекте.
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
    fn as_ptr(&self) -> *const core::ffi::c_char {
        self.0.as_ptr().cast::<core::ffi::c_char>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_are_unique_and_absolute() {
        let all = [
            SemaphoreName::RESTART_VIDEO_STREAMER,
            SemaphoreName::VIDEO_STREAMER_OVERLOAD,
            SemaphoreName::RX_RADIO_HIGH_PRIORITY,
            SemaphoreName::RX_RADIO_REG_PRIORITY,
            SemaphoreName::STARTED,
        ];
        for n in all {
            assert!(n.0.starts_with('/'), "имя должно быть абсолютным: {}", n.0);
        }
        for i in 0..all.len() {
            for j in (i + 1)..all.len() {
                assert_ne!(all[i], all[j]);
            }
        }
    }

    #[test]
    fn all_known_names_fit_and_are_valid() {
        let all = [
            SemaphoreName::RESTART_VIDEO_STREAMER,
            SemaphoreName::VIDEO_STREAMER_OVERLOAD,
            SemaphoreName::RX_RADIO_HIGH_PRIORITY,
            SemaphoreName::RX_RADIO_REG_PRIORITY,
            SemaphoreName::SM_VIDEO_DATA_AVAILABLE,
            SemaphoreName::RESTART_VEHICLE_PROCS,
            SemaphoreName::STOP_VEHICLE_ROUTER,
            SemaphoreName::STOP_VEHICLE_TELEM_TX,
            SemaphoreName::STOP_VEHICLE_COMMANDS_RX,
            SemaphoreName::STOP_VEHICLE_RC_RX,
            SemaphoreName::SET_RADIO_TX_POWER,
            SemaphoreName::SET_MAJESTIC_PARAMS_ASYNC,
            SemaphoreName::WD_CTRL_PAUSE,
            SemaphoreName::WD_CTRL_RESUME,
            SemaphoreName::WD_CTRL_RESTART,
            SemaphoreName::VIDEO_FILE_PLAYBACK_WILL_FINISH,
            SemaphoreName::VIDEO_FILE_PLAYBACK_FINISHED,
            SemaphoreName::STARTED,
        ];
        for n in all {
            assert!(n.is_valid(), "{:?} непригодно для sem_open", n.0);
            assert!(CName::new(n.0).is_some(), "{:?} не влезает в буфер", n.0);
            assert!(!n.base_name().starts_with('/'));
        }
    }

    #[test]
    fn invalid_names_are_rejected() {
        assert!(!SemaphoreName("no-slash").is_valid());
        assert!(!SemaphoreName("/a/b").is_valid());
        assert!(!SemaphoreName("/").is_valid());
        assert!(NamedSemaphore::open(SemaphoreName("bad"), SemOpen::AttachOnly, 0).is_none());
    }

    #[test]
    fn closed_semaphore_is_inert() {
        // Закрытый семафор не должен ни падать , ни «срабатывать»
        // случайно: все операции обязаны быть безопасными заглушками.
        let s = NamedSemaphore { name: SemaphoreName::STARTED, addr: core::ptr::null_mut() };
        assert!(!s.is_open());
        assert!(!s.try_signal_clear());
        assert!(!s.signal());
        assert!(!s.wait());
        assert_eq!(s.value(), -1);
        assert_eq!(s.name(), SemaphoreName::STARTED);
    }

    #[test]
    fn unlink_of_absent_semaphore_fails() {
        assert!(!NamedSemaphore::unlink(SemaphoreName("/FRUBY_TEST_NO_SUCH_SEM")));
    }

    #[test]
    fn open_of_absent_semaphore_fails() {
        // Заглушки `sem_open` всегда возвращают `SEM_FAILED`.
        let r = NamedSemaphore::open_existing(SemaphoreName("/FRUBY_TEST_NO_SUCH_SEM"));
        #[cfg(not(target_os = "linux"))]
        assert!(r.is_none());
        #[cfg(target_os = "linux")]
        let _ = r;
    }

    #[test]
    fn started_semaphore_initial_is_zero() {
        assert_eq!(STARTED_SEM_INITIAL, 0);
    }

    #[test]
    fn cname_rejects_oversized_name() {
        assert!(CName::new("/RUBY_SEM_SMALL").is_some());
        let long = format!("/{}", "S".repeat(NAME_CAPACITY + 8));
        assert!(CName::new(&long).is_none());
    }
}
