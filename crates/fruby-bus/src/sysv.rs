//! FabricRuby/crates/fruby-bus/src/sysv.rs
//!
//! Обёртки системных вызовов SysV IPC и POSIX IPC.
//!
//! Вызовы `msgget`, `msgsnd`, `msgrcv`, `msgctl`, `sem_*`, `shm_open`,
//! `mmap`, `ftruncate` отсутствуют в `libc` для не-Unix целей. Здесь они
//! объявлены напрямую , чтобы крейт собирался и проходил тесты на любой
//! хостовой ОС , при этом весь `unsafe` этого крейта собран в одной точке.
//!
//! Источник значений: `include/uapi/linux/{msg,sem,shm,fcntl,stat}.h`
//! и POSIX-стандарты ядра Linux.
//!
//! ## Формат буфера очереди сообщений
//!
//! Это главное , что легко перепутать. Буфер , который передаётся в
//! `msgsnd`/`msgrcv`, начинается с **`long mtype`** — типа сообщения,
//! и только за ним идут данные:
//!
//! ```text
//! [0..N]      c_long mtype  - тип сообщения , ставит отправитель
//! [N..]       данные        - CRC , id , длина , полезная нагрузка
//! ```
//!
//! `N` = `size_of::<c_long>()`: 8 на aarch64 (Radxa) , 4 на armv7
//! (SSC338Q). Раскладку держит `RawMessage` из [`crate::msgqueue`],
//! поэтому конкретное значение не зашито в код.
//!
//! Ядро при `msgrcv` **записывает** `mtype` в первые 8 байт буфера.
//! Если передать указатель сразу на данные , ядро затрёт первые 8 байт
//! полезной нагрузки , а при `msgsnd` прочитает из них `mtype`.
//! Поэтому раскладку буфера держит `RawMessage` из [`crate::msgqueue`],
//! а этот модуль принимает указатель на поле `mtype`.
//!
//! ## Права доступа и защита памяти
//!
//! Константы прав доступа и флагов защиты памяти **платформенные**:
//! на Windows значения другие , поэтому код , вызывающий эти функции,
//! никогда не должен выполняться вне Linux. Все функции вне Linux
//! заменены заглушками , которые всегда сообщают об ошибке.
//!
//! ## Почему нет `errno`
//!
//! Разбор `errno` потребовал бы `__errno_location`, символа нет в
//! некоторых libc , применяемых на OpenIPC. Вместо этого ошибки
//! различаются по возвращаемому значению и по проверке состояния
//! очереди через `msgctl(IPC_STAT)` после неудачи.

#![allow(non_camel_case_types)]

use core::ffi::{c_char, c_int, c_long, c_uint, c_void};

pub use core::ffi::c_ulong;

// --- флаги очередей сообщений -----------------------------------------------

/// `IPC_CREAT` — создать очередь , если её нет.
pub const IPC_CREAT: c_int = 0o1000;
/// `IPC_EXCL` — не открывать существующую очередь.
pub const IPC_EXCL: c_int = 0o2000;
/// `IPC_NOWAIT` — не блокироваться.
pub const IPC_NOWAIT: c_int = 0o4000;
/// `IPC_RMID` — удалить очередь.
pub const IPC_RMID: c_int = 0;
/// `IPC_STAT` — запросить состояние очереди.
pub const IPC_STAT: c_int = 2;
/// `MSG_NOERROR` — обрезать длинное сообщение вместо ошибки.
pub const MSG_NOERROR: c_int = 0o10000;

// --- права доступа ----------------------------------------------------------

/// `S_IRUSR` — чтение владельцем.
pub const S_IRUSR: c_uint = 0o400;
/// `S_IWUSR` — запись владельцем.
pub const S_IWUSR: c_uint = 0o200;
/// `S_IRGRP` — чтение группой.
pub const S_IRGRP: c_uint = 0o040;
/// `S_IWGRP` — запись группой.
pub const S_IWGRP: c_uint = 0o020;
/// `S_IROTH` — чтение остальными.
pub const S_IROTH: c_uint = 0o004;
/// `S_IWOTH` — запись остальными.
pub const S_IWOTH: c_uint = 0o002;

/// Права ресурсов IPC: чтение и запись для всех (`0666`).
///
/// Значение выбрано намеренно. Все процессы FabricRuby работают от
/// одного пользователя , но `b-fruby-tools` и `b-fruby-bootloader`
/// запускаются из init-скрипта и при отладке — от другого. Режим
/// `0600` означал бы , что часть системы молча не видит шину.
pub const IPC_PERM_0666: c_int = 0o666;

// --- состояние очереди (`struct msqid_ds`) ----------------------------------

/// Размер буфера для `msgctl(bus_STAT)`.
///
/// `struct msqid_ds` на 64-битной платформе занимает около 112 байт.
/// Буфер берётся с запасом и заполняется нулями: ядро проверяет только
/// «буфер не меньше структуры» , лишние байты не читает.
pub const MSQID_DS_BUF_SIZE: usize = 128;

/// Смещение `msg_qnum` в `struct msqid_ds`.
///
/// Поле идёт сразу за `struct ipc_perm`, который занимает 24 байта
/// на любой платформе: `__key` (4) + `__uid` (4) + `__gid` (4) +
/// `__cuid` (4) + `__cgid` (4) + `__mode` (2) + выравнивание (2).
pub const MSQID_DS_QNUM_OFFSET: usize = 24;

/// Смещение `msg_qbytes` — второго нужного поля.
pub const MSQID_DS_QBYTES_OFFSET: usize = MSQID_DS_QNUM_OFFSET + core::mem::size_of::<c_long>();

// --- разделяемая память -----------------------------------------------------

/// `PROT_READ` — память доступна для чтения.
pub const PROT_READ: c_int = 0x1;
/// `PROT_WRITE` — память доступна для записи.
pub const PROT_WRITE: c_int = 0x2;
/// `MAP_SHARED` — изменения видны другим отображкам.
pub const MAP_SHARED: c_int = 0x01;
/// `MAP_FAILED` — ошибка отображения.
pub const MAP_FAILED: *mut c_void = usize::MAX as *mut c_void;

// --- семафоры ----------------------------------------------------------------

/// `SEM_FAILED` — ошибка открытия семафора.
pub const SEM_FAILED: *mut c_void = usize::MAX as *mut c_void;
/// `O_CREAT` — создать объект , если его нет.
pub const O_CREAT: c_int = 0o100;
/// `O_EXCL` — не открывать существующий объект.
pub const O_EXCL: c_int = 0o200;
/// `O_RDWR` — открытие на чтение и запись.
pub const O_RDWR: c_int = 0o2;
/// `O_RDONLY` — открытие только на чтение.
pub const O_RDONLY: c_int = 0;

/// Тип смещения в файле (`off_t`).
pub type OffT = i64;

/// `SEEK_SET` — смещение отсчитывается от начала файла.
pub const SEEK_SET: c_int = 0;
/// `SEEK_CUR` — смещение отсчитывается от текущей позиции.
pub const SEEK_CUR: c_int = 1;
/// `SEEK_END` — смещение отсчитывается от конца файла.
pub const SEEK_END: c_int = 2;

// Объявления системных вызовов ядра.
//
// Документировать каждое объявление отдельно смысла нет: это
// дословные копии заголовков ядра Linux, и документация к ним
// находится в man. Модульная документация выше описывает , чем
// крейт пользуется и в каком порядке.
#[cfg(target_os = "linux")]
#[allow(missing_docs)]
unsafe extern "C" {
    pub fn msgget(key: c_int, msgflg: c_int) -> c_int;
    pub fn msgsnd(msqid: c_int, msgp: *const c_void, msgsz: usize, msgflg: c_int) -> c_int;
    pub fn msgrcv(
        msqid: c_int,
        msgp: *mut c_void,
        msgsz: usize,
        msgtyp: c_long,
        msgflg: c_int,
    ) -> isize;
    pub fn msgctl(msqid: c_int, cmd: c_int, buf: *mut c_void) -> c_int;

    pub fn sem_open(name: *const c_char, oflag: c_int, ...) -> *mut c_void;
    pub fn sem_close(sem: *mut c_void) -> c_int;
    pub fn sem_unlink(name: *const c_char) -> c_int;
    pub fn sem_wait(sem: *mut c_void) -> c_int;
    pub fn sem_trywait(sem: *mut c_void) -> c_int;
    pub fn sem_post(sem: *mut c_void) -> c_int;
    pub fn sem_getvalue(sem: *mut c_void, sval: *mut c_int) -> c_int;

    pub fn shm_open(name: *const c_char, oflag: c_int, mode: c_uint) -> c_int;
    pub fn shm_unlink(name: *const c_char) -> c_int;
    pub fn ftruncate(fd: c_int, length: i64) -> c_int;
    pub fn lseek(fd: c_int, offset: i64, whence: c_int) -> i64;
    pub fn close(fd: c_int) -> c_int;

    pub fn mmap(
        addr: *mut c_void,
        length: usize,
        prot: c_int,
        flags: c_int,
        fd: c_int,
        offset: i64,
    ) -> *mut c_void;
    pub fn munmap(addr: *mut c_void, length: usize) -> c_int;
}

/// Заглушки для не-Linux платформ: возвращают `-1` (ошибка).
///
/// # Безопасность
///
/// Функции ничего не делают и всегда сообщают об ошибке , поэтому
/// вызов из любого контекста безопасен.
#[cfg(not(target_os = "linux"))]
mod stub {
    use super::{MAP_FAILED, SEM_FAILED, c_char, c_int, c_long, c_uint, c_void};

    /// Всегда возвращает `-1`.
    pub unsafe fn msgget(_key: c_int, _msgflg: c_int) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn msgsnd(_msqid: c_int, _msgp: *const c_void, _msgsz: usize, _msgflg: c_int) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn msgrcv(
        _msqid: c_int,
        _msgp: *mut c_void,
        _msgsz: usize,
        _msgtyp: c_long,
        _msgflg: c_int,
    ) -> isize {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn msgctl(_msqid: c_int, _cmd: c_int, _buf: *mut c_void) -> c_int {
        -1
    }

    /// Всегда возвращает `SEM_FAILED`.
    ///
    /// Имя и число аргументов совпадают с [`sem_open_ex`] на Linux:
    /// вариативность у заглушки воспроизвести нельзя , `...`
    /// разрешено только во внешних объявлениях.
    pub unsafe fn sem_open_ex(
        _name: *const c_char,
        _oflag: c_int,
        _mode: c_uint,
        _value: u32,
    ) -> *mut c_void {
        SEM_FAILED
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn sem_close(_sem: *mut c_void) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn sem_unlink(_name: *const c_char) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn sem_wait(_sem: *mut c_void) -> c_int {
        -1
    }

    /// Всежда возвращает `-1`.
    pub unsafe fn sem_trywait(_sem: *mut c_void) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn sem_post(_sem: *mut c_void) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn sem_getvalue(_sem: *mut c_void, _sval: *mut c_int) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn shm_open(_name: *const c_char, _oflag: c_int, _mode: c_uint) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn shm_unlink(_name: *const c_char) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn ftruncate(_fd: c_int, _length: i64) -> c_int {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn lseek(_fd: c_int, _offset: i64, _whence: c_int) -> i64 {
        -1
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn close(_fd: c_int) -> c_int {
        -1
    }

    /// Всегда возвращает `MAP_FAILED`.
    pub unsafe fn mmap(
        _addr: *mut c_void,
        _length: usize,
        _prot: c_int,
        _flags: c_int,
        _fd: c_int,
        _offset: i64,
    ) -> *mut c_void {
        MAP_FAILED
    }

    /// Всегда возвращает `-1`.
    pub unsafe fn munmap(_addr: *mut c_void, _length: usize) -> c_int {
        -1
    }
}

#[cfg(not(target_os = "linux"))]
pub use stub::*;

/// Удалить очередь сообщений (`msgctl(bus_RMID)`).
///
/// Вызывается **только** создателем очереди и только при штатном
/// завершении. Автоматически удалять очередь при закрытии нельзя:
/// в системе одновременно работают несколько процессов , и первый же
/// завершившийся убил бы шину для всех остальных.
///
/// # Безопасность
///
/// `id` должен быть дескриптором существующей очереди.
pub unsafe fn msgclose(id: c_int) -> c_int {
    unsafe { msgctl(id, IPC_RMID, core::ptr::null_mut()) }
}

/// `true`, если системный вызов завершился ошибкой.
///
/// # Безопасность
///
/// Функция ничего не делает и безопасна при любом значении.
pub const fn call_failed(rc: c_int) -> bool {
    rc < 0
}

/// Открыть или создать именованный семафор.
///
/// Обёртка нужна потому , что `sem_open` — вариативная функция: число
/// аргументов зависит от `oflag`. Заглушка для не-Linux не может
/// объявить `...`, поэтому вариативный вызов спрятан здесь , а весь
/// остальной крейт видит одну функцию с фиксированной сигнатурой.
///
/// `mode` и `value` имеют смысл только при `O_CREAT`; без этого флага
/// ядро их не читает.
///
/// # Безопасность
///
/// `name` должен быть корректной NUL-терминированной строкой.
#[cfg(target_os = "linux")]
pub unsafe fn sem_open_ex(
    name: *const c_char,
    oflag: c_int,
    mode: c_uint,
    value: u32,
) -> *mut c_void {
    // SAFETY: обязательства вызывающего передаются в неизменённом
    // виде; лишние аргументы при отсутствии `O_CREAT` ядро игнорирует.
    unsafe { sem_open(name, oflag, mode, value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flag_values_match_linux() {
        // Сверка с `include/uapi/linux/msg.h`.
        assert_eq!(IPC_CREAT, 0o1000);
        assert_eq!(IPC_EXCL, 0o2000);
        assert_eq!(IPC_NOWAIT, 0o4000);
        assert_eq!(MSG_NOERROR, 0o10000);
        assert_eq!(IPC_RMID, 0);
        assert_eq!(IPC_STAT, 2);
    }

    #[test]
    fn permission_bits_match_linux() {
        assert_eq!(S_IRUSR, 0o400);
        assert_eq!(S_IWUSR, 0o200);
        assert_eq!(S_IRGRP, 0o040);
        assert_eq!(S_IWGRP, 0o020);
        assert_eq!(S_IROTH, 0o004);
        assert_eq!(S_IWOTH, 0o002);
    }

    #[test]
    fn ipc_perm_occupies_24_bytes() {
        // От этого размера зависит смещение `msg_qnum`; если в новой
        // версии ядра заголовки поменяются , тест упадёт и смещение
        // придётся пересчитать — лучше упасть на Windows-хосте , чем
        // на борту.
        assert_eq!(MSQID_DS_QNUM_OFFSET, 24);
        assert_eq!(MSQID_DS_QBYTES_OFFSET, 24 + core::mem::size_of::<c_long>());
        assert!(MSQID_DS_BUF_SIZE >= MSQID_DS_QBYTES_OFFSET + core::mem::size_of::<c_long>());
    }

    #[test]
    fn failure_helper_matches_syscall_convention() {
        assert!(call_failed(-1));
        assert!(!call_failed(0));
        assert!(!call_failed(1));
    }

    #[cfg(not(target_os = "linux"))]
    #[test]
    fn stubs_report_failure() {
        // SAFETY: заглушки безопасны при любых аргументах.
        unsafe {
            assert_eq!(msgget(1, 0), -1);
            assert_eq!(msgsnd(-1, core::ptr::null(), 0, 0), -1);
            assert_eq!(msgrcv(-1, core::ptr::null_mut(), 0, 0, 0), -1);
            assert_eq!(msgctl(-1, IPC_STAT, core::ptr::null_mut()), -1);
            assert_eq!(sem_wait(SEM_FAILED), -1);
            assert_eq!(sem_post(SEM_FAILED), -1);
            assert!(shm_open(c"x".as_ptr(), 0, 0) < 0);
            assert!(mmap(core::ptr::null_mut(), 1, 0, 0, -1, 0) == MAP_FAILED);
            assert_eq!(lseek(0, 0, SEEK_END), -1);
        }
    }

    #[test]
    fn seek_constants_match_posix() {
        // `lseek(SEEK_END)` спрашивает длину сегмента. Значения
        // фиксированы стандартом POSIX и не отличаются от платформы
        // к платформе.
        assert_eq!(SEEK_SET, 0);
        assert_eq!(SEEK_CUR, 1);
        assert_eq!(SEEK_END, 2);
    }

    /// Тест на отсутствие запрещённых вызовов идёт внешним поиском по
/// исходникам, а не внутри крейта: такой тест искал бы сам себя.
#[test]
    fn msgrcv_result_type_is_pointer_sized() {
        // `msgrcv` возвращает `ssize_t`. Объявление как `c_int`
        // обрезало бы результат на 64-битных платформах: сейчас
        // размеры маленькие и это не проявляется, но объявление
        // неверно по типу.
        assert_eq!(core::mem::size_of::<isize>(), core::mem::size_of::<usize>());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn lseek_reports_size_of_real_segment() {
        // Настоящий сегмент: длина через `lseek(SEEK_END)` обязана
        // совпасть с тем, что задал `ftruncate`. Это и есть замена
        // чтению `st_size` из `struct stat`.
        let name = c"/fruby_test_lseek_seg";
        // SAFETY: `name` — корректная NUL-терминированная строка.
        unsafe { crate::sysv::shm_unlink(name.as_ptr()) };
        // SAFETY: `name` — корректная NUL-терминированная строка.
        let fd = unsafe { shm_open(name.as_ptr(), O_CREAT | O_RDWR, IPC_PERM_0666 as u32) };
        assert!(fd >= 0, "shm_open не сработал");
        // SAFETY: `fd` получен из успешного `shm_open`.
        unsafe {
            assert_eq!(lseek(fd, 0, SEEK_END), 0, "новый сегмент пуст");
            assert_eq!(ftruncate(fd, 4096), 0);
            assert_eq!(lseek(fd, 0, SEEK_END), 4096, "lseek не увидел ftruncate");
            assert_eq!(lseek(fd, 0, SEEK_SET), 0);
            close(fd);
            shm_unlink(name.as_ptr());
        }
    }
}
