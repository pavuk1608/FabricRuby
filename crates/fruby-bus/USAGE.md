<!-- FabricRuby/crates/fruby-bus/USAGE.md -->

# fruby-bus - руководство для программиста

Документ описывает, как пользоваться крейтом. Отчёт о состоянии,
долги и история исправлений - в `ABOUT.md`.

Все примеры кода проверяются `cargo test`: файл подключён к
документации крейта как источник doctest'ов, поэтому пример , который
не компилируется , ломает сборку.

---

## 1. ЧТО ЭТО ЗА КРЕЙТ, ОДНИМ АБЗАЦЕМ

`fruby-bus` - транспорт между процессами одной машины. Четыре
механизма , каждый под свою задачу: очереди сообщений SysV
(`fruby_bus::MsgQueue`) для коротких сообщений с адресацией по номеру
канала , разделяемая память POSIX (`fruby_bus::SharedSegment`) для
крупных структур и видеопотока , именованные семафоры
(`fruby_bus::NamedSemaphore`) для порядка доступа к общей памяти и
слоты heartbeat (`fruby_bus::WatchdogSegment`) для супервизора.

Крейт ничего не делает с содержимым сообщений: не разбирает пакеты
(это `fruby-protocol`), не шифрует (это `fruby-crypto`), не ходит в
файлы (это `fruby-hal::Storage`), не работает по сети и не рисует
(это `fruby-render`). Почтальон, а не фабрика.

---

## 2. БЫСТРЫЙ СТАРТ

### 2.1 Маршрутизатор - создатель очередей

Маршрутизатор (`b-fruby-rt-vehicle` или `b-fruby-rt-station`) один на
систему и открывает все семь каналов. Он единственный , кто создаёт
очереди: [`Router::open_router`] вызывает `open` с политикой
`CreateOrAttach`.

```no_run
use fruby_bus::{ComponentId, Router};

fn main() -> Result<(), fruby_bus::Error> {
    // Заглушки прикладного кода - в этом примере они нужны только
    // чтобы показать форму вызовов.
    let lsm_frame = [0u8; 16];
    let rc_channels = [0u8; 12];
    fn handle_rc(_bytes: &[u8]) {}

    let mut bus = Router::open_router().expect("маршрутизатор не открыл шину");

    // Отправить по компоненту - канал выбирается сам.
    bus.send_component(ComponentId::Telemetry, &lsm_frame).unwrap();
    bus.send_component(ComponentId::Rc, &rc_channels).unwrap();

    // Принять сообщение, адресованное маршрутизатору.
    let mut buf = [0u8; 1600];
    match bus.receive_component(ComponentId::Rc, &mut buf)? {
        Some(n) => handle_rc(&buf[..n]),
        None => {} // очередь пуста - это штатно
    }
    Ok(())
}
```

### 2.2 Листовой процесс - только присоединяется

`b-fruby-central` , `b-fruby-mavlink` и `b-fruby-radio-*` очереди не
создают. Их политика - `AttachOnly`: если очередь ещё не поднята,
процесс падает сразу с понятной ошибкой , а не пишет в пустоту и не
крутит цикл.

```no_run
use fruby_bus::{ChannelId, MsgQueue, OpenPolicy};

fn main() -> Result<(), fruby_bus::Error> {
    fn parse_ltm(_bytes: &[u8]) {}

    let mut q = MsgQueue::open(ChannelId::RouterToTelemetry, OpenPolicy::AttachOnly)
        .expect("маршрутизатор ещё не поднял очередь телеметрии");

    let mut buf = [0u8; 1600];
    if let Some(n) = q.receive(&mut buf)? {
        parse_ltm(&buf[..n]);
    }
    q.send(&[0xDE, 0xAD])?; // ушло маршрутизатору
    Ok(())
}
```

### 2.3 Типы сообщений и журнал

`send` и `receive` работают типом [`MSG_TYPE_DATA`] (1).
`send_typed` и `receive_typed` позволяют разделить трафик: `b-fruby-tools`
читает только [`MSG_TYPE_LOG`] (2) и не разбирает содержимое.

```no_run
use fruby_bus::{ChannelId, MsgQueue, MSG_TYPE_LOG, OpenPolicy};

fn main() -> Result<(), fruby_bus::Error> {
    let mut tx = MsgQueue::open(ChannelId::RouterToCentral, OpenPolicy::AttachOnly)?;

    // Текст с не-ASCII байтами - это `str` , а не `b"..."`:
    // байтовый литерал допускает только ASCII.
    let line = "радио: потерял 12 пакетов\n";
    tx.send_typed(MSG_TYPE_LOG, line.as_bytes())?;
    Ok(())
}
```

Приём только журнала:

```no_run
use fruby_bus::{ChannelId, MsgQueue, OpenPolicy};

fn main() -> Result<(), fruby_bus::Error> {
    fn print_log(_bytes: &[u8]) {}

    let mut rx = MsgQueue::open(ChannelId::RouterToCentral, OpenPolicy::AttachOnly)?;
    let mut line = [0u8; 512];
    while let Some(n) = rx.receive_log(&mut line)? {
        print_log(&line[..n]);
    }
    Ok(())
}
```

### 2.4 Разделяемая память с повторами попытки

Писатель создаёт объект и лишь затем задаёт его длину. Читатель ,
пришедший в это окно , увидит длину `0` и получит `None`. Семь
процессов стартуют одновременно , поэтому окно реально. Листовым
процессам положено пользоваться
[`SharedSegment::open_read_retry`].

```no_run
use fruby_bus::{SegmentName, SharedSegment, watchdog};

// 50 попыток по 20 мс - до секунды на ожидание.
let seg = SharedSegment::open_read_retry(
    SegmentName::WATCHDOG,
    watchdog::LAYOUT.size,
    50,
    20,
)
.expect("сегмент heartbeat не появился за 1 с");
# let _ = seg;
```

### 2.5 Heartbeat: процесс и супервизор

Процесс один раз открывает сегмент и пишет знак в цикле. Время
передаётся снаружи: у крейта нет часов, `UptimeMillis` приходит от
`fruby-hal::Clock` процесса.

Знак никогда не пишется нулём: в первые 500 мс после загрузки аптайм
равен нулю , и запись нуля означала бы «никогда не писал».

```no_run
use fruby_bus::{Ownership, UptimeMillis, WatchdogSegment, watchdog};

fn main() -> Result<(), fruby_bus::Error> {
    let wd = WatchdogSegment::open(Ownership::Writer).expect("сегмент heartbeat");

    // `now` берётся из часов процесса, а не из крейта.
    let now = UptimeMillis(1_000);
    loop {
        // основная работа процесса
        wd.heartbeat(watchdog::SlotRole::VehicleRouter, now);
    }
}
```

Супервизор читает те же слоты. Слот , который ни разу не писал ,
считается мёртвым при любом времени - значит , перезапускать по нему
нельзя. Стартовое окно выбирает загрузчик , не крейт.

```no_run
use fruby_bus::{Ownership, UptimeMillis, WatchdogSegment, watchdog};

fn main() -> Result<(), fruby_bus::Error> {
    let wd = WatchdogSegment::open(Ownership::Reader).expect("сегмент heartbeat");
    let now = UptimeMillis(10_000);
    let stale = wd.stale_roles(now, 3_000);
    for role in stale {
        println!("{} не отвечает", role.bin_name());
    }
    Ok(())
}
```

### 2.6 Семафор

Семафоры хранят признаки , а не данные. Типичный ход: отправитель
поднимает семафор , получатель опрашивает его без блокировки и сбрасывает
сигнал.

```no_run
use fruby_bus::{NamedSemaphore, SemOpen, SemaphoreName};

fn main() -> Result<(), fruby_bus::Error> {
    fn do_other_work() {}
    fn apply_tx_power() {}

    // Создатель один раз поднимает сигнал.
    let s = NamedSemaphore::open(
        SemaphoreName::SET_RADIO_TX_POWER,
        SemOpen::CreateOrAttach,
        0,
    )
    .expect("семафор мощности");

    s.signal(); // «пора менять мощность»

    // Получатель в своём цикле опрашивает без блокировки.
    while !s.try_signal_clear() {
        do_other_work();
    }
    apply_tx_power();
    Ok(())
}
```

---

## 3. КАНАЛЫ И МАРШРУТИЗАЦИЯ

### 3.1 Номера каналов

Значения совпадают с `ruby_bus.h` и **не меняются**: они попадают в
заголовок сообщения и в лог , и на них рассчитывают внешние утилиты
(см. `ASSUMPTIONS.md`).

| Канал | Код | Имя | Кто пишет | Кто читает |
|---|---:|---|---|---|
| `RouterToRc` | 68 | `router_to_rc` | `b-fruby-central` | `b-fruby-radio-*` |
| `RcToRouter` | 69 | `rc_to_router` | `b-fruby-radio-*` | `b-fruby-rt-*` |
| `RouterToCentral` | 70 | `router_to_central` | `b-fruby-rt-station` | `b-fruby-central` |
| `CentralToRouter` | 71 | `central_to_router` | `b-fruby-central` | `b-fruby-rt-station` |
| `RouterToCommands` | 77 | `router_to_commands` | `b-fruby-central` | `b-fruby-rt-*` |
| `CommandsToRouter` | 78 | `commands_to_router` | `b-fruby-rt-*` | `b-fruby-central` |
| `CsiVideoCommands` | 81 | `csi_video_commands` | `b-fruby-central` | `b-fruby-rt-vehicle` |
| `RouterToTelemetry` | 83 | `router_to_telemetry` | `b-fruby-central` | `b-fruby-mavlink` |
| `TelemetryToRouter` | 84 | `telemetry_to_router` | `b-fruby-mavlink` | `b-fruby-rt-station` |

Маршрутизатор открывает семь каналов из девяти -
[`ChannelId::ROUTER_CHANNELS`]. Видео в SysV-очередь не идёт.

**Про имена каналов.** Имена унаследованы от C и не отражают реальных
участников: `RouterToRc` (68) пишет не router , а `b-fruby-central`;
`RouterToCommands` (77) - тоже `central`. Менять имена нельзя: они
пишутся в лог и на них рассчитывают внешние утилиты. Ориентироваться
надо на столбцы «Кто пишет» и «Кто читает» , а не на название.

### 3.2 Кто создаёт очередь

| Роль | Политика | Поведение при отсутствии очереди |
|---|---|---|
| маршрутизатор (`rt-vehicle` , `rt-station`) | `CreateOrAttach` | создаёт |
| `b-fruby-central` | `AttachOnly` | падает |
| `b-fruby-mavlink` | `AttachOnly` | падает |
| `b-fruby-radio-*` | `AttachOnly` | падает |
| `b-fruby-tools` | `AttachOnly` | падает |

Гонки при одновременном старте нет: `CreateOrAttach` при `EEXIST`
повторяет `msgget` без `IPC_EXCL` и присоединяется. Порядок запуска
не имеет значения.

### 3.3 Компонент → канал

[`ChannelFor::for_component`] выбирает канал по компоненту пакета и
направлению. У части компонентов канала **нет** - они возвращают
`None` , и [`Router::send_component`] в этом случае возвращает
`Error::Unsupported`.

| Компонент | От маршрутизатора | К маршрутизатору |
|---|---|---|
| `Rc` | `RouterToRc` (68) | `RcToRouter` (69) |
| `Telemetry` | `RouterToTelemetry` (83) | `TelemetryToRouter` (84) |
| `Commands` | `RouterToCommands` (77) | `CommandsToRouter` (78) |
| `Video` | - | - |
| `LocalControl` | - | - |
| `Audio` | - | - |
| `Ruby` | - | - |
| `Reserved` | - | - |

Видео идёт через разделяемую память; `LocalControl` не выходит из
процесса; служебные сообщения `Ruby` разбирает сам маршрутизатор.

---

## 4. ФОРМАТ СООБЩЕНИЯ

Очередь SysV начинается с `long mtype`, который пишет ядро. За ним -
данные в раскладке [`busMessage`]:

```text
[0..4)   u32   CRC32 по байтам [4 .. 7+len)
[4]      u8    message id (монотонный , переполняется на 256)
[5]      u8    младший байт длины полезной нагрузки
[6]      u8    старший байт длины полезной нагрузки
[7..)    ...   полезная нагрузка
```

- [`MAX_MSG_SIZE`] = `1600` - ровно как `char data[1600]` в C;
- [`msgqueue::PAYLOAD_CAPACITY`] = `1593` - максимум полезной нагрузки;
- [`MSG_TYPE_DATA`] = `1`, [`MSG_TYPE_LOG`] = `2`.

CRC обязателен: очередь не гарантирует целостность. Идентификатор
нужен для подсчёта потерь: [`MsgQueue::receive`] сам сравнивает `id` с
предыдущим и пишет разницу в [`ChannelStats::lost`].

Разрыв больше половины диапазона (128) неразличим с поломкой счётчика.
В этом случае число потерянных сообщений **не выдумывается** , а растёт
счётчик [`ChannelStats::unordered`].

Пустой разрыв и потери:

```no_run
use fruby_bus::busMessage;

let prev = busMessage::new(254);
let next = busMessage::new(2);
assert_eq!(next.gap_after(prev.id), Some(4)); // пропущены 255, 0, 1

let wild = busMessage::new(60);
assert_eq!(wild.gap_after(100), None); // разрыв неразличим
```

---

## 5. РАЗДЕЛЯЕМАЯ ПАМЯТЬ

### 5.1 Сегменты

| Раскладка | Сегмент | Размер | Сверено с C |
|---|---|---:|---|
| `layouts::CONTROLLER_ONLY` | `SYSTEM_SHARED_MEM_CONTROLLER_ONLY` | 4096 | **нет** |
| `layouts::I2C` | `SYSTEM_SHARED_MEM_RUBY_I2C_DATA` | 1024 | **нет** |
| `layouts::RADIO` | `SYSTEM_SHARED_MEM_RUBY_RADIO_DATA` | 2048 | **нет** |
| `layouts::COMMON` | `SYSTEM_SHARED_MEM_COMMON` | 65536 | **нет** |
| `watchdog::LAYOUT` | `SYSTEM_SHARED_MEM_WATCHDOG` | 736 | **да** |

Четыре первых размера **предположительные**: исходников C в этом
workspace нет. У таких раскладок [`ShmLayout::verified`] = `false` , а
[`layouts::has_unverified`] возвращает `true`.

Что это значит на практике: при переходном периоде , когда часть
процессов ещё собрана из оригинала , процессы с разной раскладкой не
поймут друг друга. `b-fruby-tools` обязан помечать неподтверждённые
сегменты в журнале. Снять флаг можно только после сверки размеров с
исходником.

Единственная сверенная раскладка - `WATCHDOG`: 736 байт выводится из
23 полей `u32` по 4 байта на слот и восьми слотов , то есть не зависит
от содержимого типов C.

### 5.2 Правила

- **Обнуление только при создании.** Повторное открытие на запись не
  стирает накопленное. В C любой процесс , открывший сегмент на
  запись , обнулял его целиком.
- **Удаляет создатель**, явно: [`SharedSegment::unlink`]. Само
  отображение у уже работающих процессов при этом остаётся живым.
- **Длина спрашивается `lseek(SEEK_END)`** , а не чтением размера из
  `struct stat`: у `stat` две несовместимые раскладки (glibc и
  `asm-generic`), и объявленная «средняя» структура читала бы мусор на
  aarch64 - это станция Radxa 3W.
- **Имя проверяется на длину до копирования.** `heapless` при
  переполнении тихо обрезает , и обрезанное имя указало бы на чужой
  объект в `/dev/shm`.

---

## 6. WATCHDOG

Сегмент `SYSTEM_SHARED_MEM_WATCHDOG` - восемь слотов по 92 байта.
Слот = `#[repr(C)]` из 23 `u32` (8 именованных , 15 зарезервированы),
что совпадает с `shared_mem_process_stats` побайтно.

| Индекс | Роль | Процесс |
|---:|---|---|
| 0 | `VehicleRouter` | `b-fruby-rt-vehicle` |
| 1 | `VehicleRadio` | `b-fruby-radio-vehicle` |
| 2 | `Mavlink` | `b-fruby-mavlink` |
| 3 | `StationRouter` | `b-fruby-rt-station` |
| 4 | `Central` | `b-fruby-central` |
| 5 | `StationRadio` | `b-fruby-radio-station` |
| 6 | `Tools` | `b-fruby-tools` |
| 7 | `Bootloader` | `b-fruby-bootloader` |

Нумерация фиксирована: адрес слота лежит в разделяемой памяти и
переживает перезапуск процессов.

Поля слота, доступные прикладному коду:
[`last_active`](WatchdogSlot::last_active) ,
`loop_counter` , `max_loop_time_ms` , `avg_loop_time_ms` ,
`alarm_flags` , `alarm_time`.

Поведение пустых слотов:

- слот , который ни разу не писал , **мёртв** при любом
  [`WatchdogSlot::is_alive`] - см. [`WatchdogSlot::has_heartbeat`];
- [`heartbeat`] при нулевом аптайме пишет единицу , а не ноль;
- [`stale_roles`] не возвращает `Bootloader`: супервизор не может
  перезапустить самого себя по своему же слоту.

```no_run
use fruby_bus::{Ownership, UptimeMillis, WatchdogSegment, watchdog};

let wd = WatchdogSegment::open(Ownership::Writer).expect("сегмент");
let slot = wd.slot_of(watchdog::SlotRole::Central).expect("слот 4");
assert!(!slot.has_heartbeat(), "только что создали - никто не писал");

wd.heartbeat(watchdog::SlotRole::Central, UptimeMillis(1_000));
assert!(slot.is_alive(UptimeMillis(1_100), 500));
assert!(!slot.is_alive(UptimeMillis(5_000), 500));
```

---

## 7. ЖИЗНЕСНЕННЫЙ ЦИКЛ И ОШИБКИ

### 7.1 Кто создаёт и кто удаляет

| Ресурс | Создаёт | Удаляет | Автоудаление |
|---|---|---|---|
| очередь SysV | маршрутизатор | создатель , `MsgQueue::remove` | **нет** |
| сегмент shm | первый писатель , `Ownership::Writer` | создатель , `SharedSegment::unlink` | **нет** |
| семафор | первый писатель , `SemOpen::CreateOrAttach` | создатель , `NamedSemaphore::unlink` | **нет** |

Автоудаления нет , и это не упущение. Первый завершившийся процесс
убил бы шину для всех остальных: маршрутизатор умер , а
`b-fruby-central` , `b-fruby-mavlink` и радио молча крутили бы цикл на
мёртвой шине. Поэтому удаление - явное действие создателя.

Исключение одно: [`Router::open`] при неудачном открытии снимает
только те очереди , которые создал сам ([`MsgQueue::was_created`]).
Присоединённые очереди принадлежат маршрутизатору и должны пережить
чужую ошибку.

### 7.2 Что возвращает метод

| Ситуация | Что вернёт | Что делать вызывающему |
|---|---|---|
| очередь пуста | `Ok(None)` | ничего , это штатно |
| очередь удалена при приёме | `Err(Error::InvalidState)` | остановиться , шина мертва |
| очередь удалена при отправке | `Err(Error::InvalidState)` | то же |
| очередь полна | `Err(Error::OutOfRange)` , счётчик `dropped` | счётчик `lost` покажет цену |
| полезная нагрузка больше 1593 | `Err(Error::OutOfRange)` | уменьшить пакет |
| CRC не сошёлся | `Err(Error::BadChecksum)` , счётчик `corrupted` | пропустить сообщение |
| длина в заголовке не бьётся с буфером | `Err(Error::Truncated)` | то же |
| сегмент короче раскладки | `None` | несовпадение версий , не стартовать |
| сегмента ещё нет | `None` | повторить , см. `open_read_retry` |
| раскладка не сверена с C | `validate_layout` вернёт `Error::InvalidState` | предупредить в журнал |

Про различение «очередь удалена» и «очередь пуста»: `msgrcv` и `msgsnd`
возвращают один и тот же код и для пустоты , и для удаления.
Крейт различает их вызовом msgctl(IPC_STAT) после неудачи: на живой
очереди он отвечает , на удалённой - нет.

Плата за это решение - один лишний системный вызов на каждый неудачный
приём. В цикле с плотным трафиком это незаметно; в цикле опроса пустой
очереди заметно. Обратный вариант (проверять IPC_STAT до приёма)
стоил бы системного вызова на каждое принятое сообщение и
не давал бы ничего сверх того , что даёт проверка после.

### 7.3 Права доступа

Очереди , сегменты и семафоры создаются с правами `0666`. Все процессы
FabricRuby работают от одного пользователя , но `b-fruby-tools` и
`b-fruby-bootloader` запускаются из init-скрипта и при отладке - от
другого. Режим `0600` означал бы , что часть системы молча не видит
шину.

---

## 8. ЧЕГО КРЕЙТ НЕ ДЕЛАЕТ

| Задача | Кто отвечает |
|---|---|
| разбор и сборка пакетов | `fruby-protocol` |
| шифрование , FEC | `fruby-crypto` |
| чтение и запись файлов | `fruby-hal::Storage` |
| радиолиния , AF_PACKET | `fruby-hal::RadioInterface` |
| рисование , DRM/KMS | `fruby-render` , `fruby-ui` |
| управление процессами | `fruby-hal::System` , загрузчик |
| работа по сети | нигде в FabricRuby |

Крейт не знает ни о ролях , ни о бинарниках. Роли подставляют снаружи
через [`ChannelFor`] и `fruby_daemon_common::ipc_daemon::channels_for`.

---

## 9. ТЕСТИРОВАНИЕ

### 9.1 Что проверяют тесты на Windows

Все системные вызовы под `cfg(not(target_os = "linux"))` заменены
заглушками , которые всегда возвращают ошибку. Поэтому тесты
проверяют:

- раскладку сообщения и CRC на настоящих байтах;
- ключи очередей , их положительность и уникальность;
- логику счётчиков: потери , разрывы идентификаторов , слияние;
- раскладки сегментов , имена , проверку длин;
- поведение слота heartbeat , включая нулевой аптайм и переполнение
  32-битного счётчика;
- откат очередей при неудачном `Router::open`;
- инерность закрытых семафоров и отказ на непригодных именах.

### 9.2 Чего тесты не проверяют

Заглушки **не проверяют поведение ядра**. Конкретно: реальную
доставку сообщения между процессами , реальное разделение памяти ,
реальные права `0666` , реальное различение `EAGAIN` и `EIDRM`
(оно построено на `msgctl` , и заглушка его не воспроизводит).

### 9.3 Linux

Linux-ветка проверяется отдельно и обязательно. Она содержит тесты на
настоящих ресурсах:

- `linux_roundtrip_through_real_queue` - создать , отправить ,
  принять , снять;
- `linux_receive_reports_removed_queue_as_dead` - удалённая очередь
  даёт `Err` , а не «пусто»;
- `linux_full_queue_is_reported_as_drop_not_error` - переполнение
  попадает в `dropped` , а не в `errors`;
- `linux_segment_is_created_zeroed_once_and_unlinked` - обнуление
  ровно один раз , удаление по `unlink`;
- `linux_reader_rejects_segment_shorter_than_layout`;
- `linux_reader_retry_waits_for_writer` - повтор дожидается писателя;
- `lseek_reports_size_of_real_segment` - длина через `lseek`.

Тесты , работающие с системными ресурсами , сериализованы: имена SysV
общие для всего пользователя , и параллельный запуск двух тестов на
одном канале мешал бы друг другу.

### 9.4 Как смотреть живую систему

На живой машине после запуска:

```sh
# очереди сообщений: ключ, идентификатор, счётчики, права
ipcs -q

# сегменты разделяемой памяти
ipcs -m

# что реально лежит в /dev/shm
ls -l /dev/shm

# предел очереди в байтах (именно он меряется в QueueStat)
cat /proc/sys/kernel/msgmnb

# семафоры
ipcs -s
```

Ожидаемое после штатного завершения: очереди и сегменты , созданные
процессом , исчезают из вывода. Если остались - процесс не дошёл до
явного удаления - это дефект. Чистить вручную:
`ipcrm -q <id>` и `ipcrm -m <id>`.