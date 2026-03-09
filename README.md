# gnss_io_helper

A lightweight, SOLID-designed C++ I/O channel library for Linux, targeting automotive embedded systems — in particular, GNSS/UBX hardware communication in Location module applications.

The library abstracts **UART (termios)**, **I2C / u-blox DDC**, and **binary file record/replay** behind a single polymorphic interface, so the GNSS stack never needs to know which physical channel is in use.

---

## Features

- **Multi-transport** — UART (POSIX termios), I2C / u-blox DDC (Linux i2c-dev), and binary file record/replay all implement the same `i_io_port` interface.
- **SOLID architecture** — segregated `i_io_reader` / `i_io_writer` interfaces, polymorphic `i_io_config` base, open for new transports (SPI, TCP …) without touching existing code.
- **Factory pattern** — `io_port_factory::create(config)` selects the correct concrete port based on `config.channel_type()`; callers depend only on abstractions.
- **Thread-safe implementation** — every concrete port (`uart_port`, `i2c_port`, `file_port`) guards its file descriptor behind an internal `std::mutex`; safe to share between reader and writer threads.
- **High-level session wrapper** — `io_channel_wrapper` owns the port via `shared_ptr` / `weak_ptr`, exposes lifecycle (`initialize`, `shutdown`, `reconnect`, `reconfigure`) and forwards all I/O calls.
- **fd accessor** — `get_fd()` exposes the underlying POSIX file descriptor for `poll()` / `epoll()` / `select()` use (returns -1 for I2C, which is not pollable).
- **Rich UART configuration** — all standard baud rates (50 bps → 4 Mbps), data bits, parity, stop bits, hardware/software flow control, blocking and non-blocking modes.
- **I2C / u-blox DDC** — combined `I2C_RDWR` transactions for register-addressed read, 16-bit `available()` counter via registers `0xFD`/`0xFE`, write via register `0x00`.
- **File record/replay** — read-only replay (with optional looping and inter-read throttle to simulate real-time throughput), or read-write recording mode.
- **Structured exception hierarchy** — base `io_exception` → per-operation bases (`io_open_error`, `io_config_error`, `io_read_error`, `io_write_error`, `io_timeout_error`) → per-transport subtypes (`uart_open_error`, `i2c_read_error`, `file_open_error`, …).
- **C++17**, no external dependencies beyond the C++ standard library and POSIX / Linux kernel headers.

---

## Repository Layout

```
gnss_io_helper/
├── i_io_config.h              # Polymorphic config base + io_channel_type enum
├── i_io_reader.h              # ISP: read-only interface
├── i_io_writer.h              # ISP: write-only interface
├── i_io_port.h                # Full-duplex interface: reader + writer + lifecycle + fd accessor
├── io_exception.h             # Base exception hierarchy (io_exception + 5 base error types)
├── io_port_factory.h/.cpp     # Factory: create(config) → shared_ptr<i_io_port>
├── io_channel_wrapper.h/.cpp  # High-level session wrapper (DI, weak_ptr to port)
│
├── uart/
│   ├── uart_types.h           # Enums: baud_rate, data_bits, parity, stop_bits, flow_control
│   ├── uart_config.h          # uart_config struct + factory helpers (gnss_default, gnss_high_speed …)
│   ├── uart_exception.h       # uart_open_error, uart_config_error, uart_read_error, uart_write_error
│   └── uart_port.h/.cpp       # Concrete POSIX termios implementation of i_io_port
│
├── i2c/
│   ├── i2c_config.h           # i2c_config struct + DDC register constants + factory helpers
│   ├── i2c_exception.h        # i2c_open_error, i2c_config_error, i2c_read_error
│   └── i2c_port.h/.cpp        # Concrete Linux i2c-dev implementation of i_io_port
│
├── file/
│   ├── file_config.h          # file_config struct + replay / record / custom factory helpers
│   ├── file_exception.h       # file_open_error, file_read_error
│   └── file_port.h/.cpp       # Concrete POSIX file implementation of i_io_port
│
└── architecture.puml          # PlantUML class + sequence diagrams
```

---

## Architecture Overview

```
gnss_controller  (consumer)
       │
       │  depends on abstraction only
       ▼
io_channel_wrapper          ← holds weak_ptr<i_io_port>; lifecycle + I/O delegation
       │
       ▼
  i_io_port                 ← i_io_reader + i_io_writer + open/close/flush/drain/reconnect/reconfigure/get_fd
       ▲
       │  created by
io_port_factory::create(config)
       │
       ├── uart_port         ← POSIX termios  (uart_config)
       ├── i2c_port          ← Linux i2c-dev  (i2c_config, u-blox DDC)
       └── file_port         ← POSIX file     (file_config, replay / record)
```

Consumers that only receive data depend on `i_io_reader`; those that only send depend on `i_io_writer`. The full `i_io_port` is used internally by each concrete implementation. High-level components interact exclusively through `io_channel_wrapper`.

---

## Transport Quick-Reference

| Transport | Config type | Key fields | `get_fd()` | `available()` mechanism |
|-----------|-------------|-----------|------------|------------------------|
| UART | `uart_config` | `device_path`, `baud`, data/parity/stop/flow, `read_timeout_ms`, `non_blocking` | POSIX tty fd | `ioctl(FIONREAD)` |
| I2C / DDC | `i2c_config` | `device_path`, `address` (default 0x42), `read_timeout_ms` | -1 (not pollable) | DDC registers 0xFD/0xFE via `I2C_RDWR` |
| File | `file_config` | `file_path`, `writable`, `loop`, `throttle_us` | POSIX file fd | `fstat` + `lseek(SEEK_CUR)` |

---

## Exception Hierarchy

```
std::runtime_error
└── io_exception
    ├── io_open_error
    │   ├── uart_open_error
    │   ├── i2c_open_error
    │   └── file_open_error
    ├── io_config_error
    │   ├── uart_config_error
    │   └── i2c_config_error
    ├── io_read_error
    │   ├── uart_read_error
    │   ├── i2c_read_error
    │   └── file_read_error
    ├── io_write_error
    │   └── uart_write_error
    └── io_timeout_error
```

Catch at any granularity. A generic `catch (const io::io_exception& e)` handles all library errors.

---

## Usage Examples

### UART (u-blox GNSS default 9600-8N1)

```cpp
#include "io_port_factory.h"
#include "io_channel_wrapper.h"
#include "uart/uart_config.h"

auto cfg  = io::uart_config::gnss_default("/dev/ttyS0");
auto port = io::io_port_factory::create(cfg);
io::io_channel_wrapper ch(port);
ch.initialize(cfg);

std::vector<uint8_t> frame;
ch.read_bytes(frame, 256);
ch.shutdown();
```

### Switch to 115200 bps after UBX-CFG-PRT command

```cpp
ch.reconfigure(io::uart_config::gnss_high_speed("/dev/ttyS0"));
```

### I2C / u-blox DDC

```cpp
#include "io_port_factory.h"
#include "io_channel_wrapper.h"
#include "i2c/i2c_config.h"

auto cfg  = io::i2c_config::ddc_default("/dev/i2c-1");   // address 0x42
auto port = io::io_port_factory::create(cfg);
io::io_channel_wrapper ch(port);
ch.initialize(cfg);

int n = ch.available();    // queries DDC registers 0xFD / 0xFE
if (n > 0) {
    std::vector<uint8_t> buf;
    ch.read_bytes(buf, static_cast<std::size_t>(n));
}
```

### File replay (offline testing)

```cpp
#include "io_port_factory.h"
#include "io_channel_wrapper.h"
#include "file/file_config.h"

// Replay a recorded binary capture, looping indefinitely at real-time speed
auto cfg  = io::file_config::replay("/data/gnss_capture.bin",
                                    /*loop=*/true, /*throttle_us=*/1000);
auto port = io::io_port_factory::create(cfg);
io::io_channel_wrapper ch(port);
ch.initialize(cfg);

std::vector<uint8_t> buf;
while (true) {
    if (ch.available() > 0)
        ch.read_bytes(buf, 512);
}
```

### Extending with a new transport (e.g. SPI)

1. Add a `spi/` sub-folder with `spi_config.h`, `spi_exception.h`, `spi_port.h/.cpp`.
2. Add `io_channel_type::spi` to the `io_channel_type` enum in `i_io_config.h`.
3. Add one `case io_channel_type::spi:` in `io_port_factory.cpp`.
4. Zero other files change.

---

## Building

```bash
# Development host (x86-64 Linux)
g++ -std=c++17 -Wall -Wextra -o gnss_io_helper \
    io_port_factory.cpp io_channel_wrapper.cpp \
    uart/uart_port.cpp i2c/i2c_port.cpp file/file_port.cpp \
    your_main.cpp

# Cross-compile for an aarch64 Yocto target
CXX=aarch64-poky-linux-g++ \
g++ -std=c++17 -Wall -Wextra -O2 --sysroot=<SDK_SYSROOT> \
    io_port_factory.cpp io_channel_wrapper.cpp \
    uart/uart_port.cpp i2c/i2c_port.cpp file/file_port.cpp \
    your_main.cpp -o gnss_io_helper
```

**Requirements:** GCC/G++ 7+ with C++17 support, Linux kernel 4.x or later.

---

## License

MIT — see [LICENSE](LICENSE).
