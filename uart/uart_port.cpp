/**
 * MIT License
 *
 * Copyright (c) 2026 nguyenchiemminhvu@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * uart_port.cpp
 * ============================================================
 * Linux POSIX termios implementation of uart_port (i_io_port).
 *
 * Requires: Linux kernel 4.x+ (Yocto / embedded target).
 * Build:    g++ -std=c++14 -c uart_port.cpp -o uart_port.o
 */

#include "uart_port.h"
#include "uart_exception.h"

#include <fcntl.h>       // open, O_RDWR, O_NOCTTY, O_NONBLOCK
#include <unistd.h>      // read, write, close
#include <sys/ioctl.h>   // ioctl
#include <termios.h>     // termios, cfsetispeed, cfsetospeed, tcsetattr …
#include <cerrno>
#include <cstring>       // strerror
#include <utility>       // std::move, std::swap

// Linux-specific ioctl to query bytes waiting in the RX kernel buffer
#ifndef FIONREAD
    #include <sys/filio.h>   // fallback (macOS / BSDs)
#endif

#include <iostream>

namespace io
{

// ============================================================
//  Constructor / Destructor
// ============================================================

uart_port::uart_port()
    : fd_(-1)
{
}

uart_port::~uart_port()
{
    if (fd_ >= 0)
    {
        ::tcflush(fd_, TCIOFLUSH);
        ::close(fd_);
        fd_ = -1;
    }
}

// ============================================================
//  Move semantics
// ============================================================

uart_port::uart_port(uart_port&& other) noexcept
{
    std::lock_guard<std::mutex> lock(other.mutex_);
    fd_       = other.fd_;
    config_   = std::move(other.config_);
    other.fd_ = -1;
}

uart_port& uart_port::operator=(uart_port&& other) noexcept
{
    if (this != &other)
    {
        // Lock both mutexes in a consistent order to avoid deadlock.
        std::unique_lock<std::mutex> lhs_lock(mutex_,       std::defer_lock);
        std::unique_lock<std::mutex> rhs_lock(other.mutex_, std::defer_lock);
        std::lock(lhs_lock, rhs_lock);

        if (fd_ >= 0)
        {
            ::tcflush(fd_, TCIOFLUSH);
            ::close(fd_);
        }
        fd_       = other.fd_;
        config_   = std::move(other.config_);
        other.fd_ = -1;
    }
    return *this;
}

// ============================================================
//  Lifecycle
// ============================================================

bool uart_port::open(const i_io_config& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return open_locked(config);
}

void uart_port::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
}

bool uart_port::is_open() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fd_ >= 0;
}

const i_io_config& uart_port::get_config() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

bool uart_port::flush()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        return false;
    }
    return ::tcflush(fd_, TCIOFLUSH) == 0;
}

bool uart_port::drain()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        return false;
    }
    return ::tcdrain(fd_) == 0;
}

bool uart_port::reconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        return false;
    }
    const uart_config saved = config_;
    close_locked();
    return open_locked(saved);
}

bool uart_port::reconfigure(const i_io_config& new_config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
    return open_locked(new_config);
}

// ============================================================
//  Private lifecycle helpers  (called with mutex_ already held)
// ============================================================

bool uart_port::open_locked(const i_io_config& config)
{
    if (config.channel_type() != io_channel_type::uart)
    {
        throw uart_config_error(
            "uart_port::open: supplied config is not a uart_config "
            "(channel_type mismatch)");
    }

    const uart_config& ucfg = static_cast<const uart_config&>(config);

    if (fd_ >= 0)
    {
        close_locked();
    }

    int flags = O_RDWR | O_NOCTTY;
    if (ucfg.non_blocking)
    {
        flags |= O_NONBLOCK;
    }

    fd_ = ::open(ucfg.device_path.c_str(), flags);
    if (fd_ < 0)
    {
        throw uart_open_error(
            "Cannot open '" + ucfg.device_path + "': " + std::strerror(errno));
    }

    config_ = ucfg;

    try
    {
        apply_termios_config();
    }
    catch (const io_exception& e)
    {
        std::cout << "Error applying termios config: " << e.what() << std::endl;
        ::close(fd_);
        fd_ = -1;
        throw;
    }

    return true;
}

void uart_port::close_locked()
{
    if (fd_ >= 0)
    {
        ::tcflush(fd_, TCIOFLUSH);
        ::close(fd_);
        fd_ = -1;
    }
}

// ============================================================
//  i_io_reader
// ============================================================

ssize_t uart_port::read_byte(char& c)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_read_error("read_byte: port is not open");
    }
    ssize_t n = ::read(fd_, &c, 1);
    if (n < 0)
    {
        throw uart_read_error(std::string("read_byte failed: ") + std::strerror(errno));
    }
    return n;
}

ssize_t uart_port::read_chars(char* buf, std::size_t len)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_read_error("read_chars: port is not open");
    }
    if (buf == nullptr || len == 0)
    {
        return 0;
    }
    ssize_t n = ::read(fd_, buf, len);
    if (n < 0)
    {
        throw uart_read_error(std::string("read_chars failed: ") + std::strerror(errno));
    }
    return n;
}

ssize_t uart_port::read_uchars(unsigned char* buf, std::size_t len)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_read_error("read_uchars: port is not open");
    }
    if (buf == nullptr || len == 0)
    {
        return 0;
    }
    ssize_t n = ::read(fd_, buf, len);
    if (n < 0)
    {
        throw uart_read_error(std::string("read_uchars failed: ") + std::strerror(errno));
    }
    return n;
}

ssize_t uart_port::read_string(std::string& str, std::size_t max_len)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_read_error("read_string: port is not open");
    }
    if (max_len == 0)
    {
        str.clear();
        return 0;
    }
    str.resize(max_len);
    ssize_t n = ::read(fd_, &str[0], max_len);
    if (n < 0)
    {
        str.clear();
        throw uart_read_error(std::string("read_string failed: ") + std::strerror(errno));
    }
    str.resize(static_cast<std::size_t>(n));
    return n;
}

ssize_t uart_port::read_bytes(std::vector<uint8_t>& buf, std::size_t max_len)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_read_error("read_bytes: port is not open");
    }
    if (max_len == 0)
    {
        buf.clear();
        return 0;
    }
    buf.resize(max_len);
    ssize_t n = ::read(fd_, buf.data(), max_len);
    if (n < 0)
    {
        buf.clear();
        throw uart_read_error(std::string("read_bytes failed: ") + std::strerror(errno));
    }
    buf.resize(static_cast<std::size_t>(n));
    return n;
}

int uart_port::available() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        return -1;
    }
    int bytes = 0;
    if (::ioctl(fd_, FIONREAD, &bytes) < 0)
    {
        return -1;
    }
    return bytes;
}

// ============================================================
//  i_io_writer
// ============================================================

ssize_t uart_port::write_byte(char c)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_write_error("write_byte: port is not open");
    }
    ssize_t n = ::write(fd_, &c, 1);
    if (n < 0)
    {
        throw uart_write_error(std::string("write_byte failed: ") + std::strerror(errno));
    }
    return n;
}

ssize_t uart_port::write_chars(const char* buf, std::size_t len)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_write_error("write_chars: port is not open");
    }
    if (buf == nullptr || len == 0)
    {
        return 0;
    }
    ssize_t n = ::write(fd_, buf, len);
    if (n < 0)
    {
        throw uart_write_error(std::string("write_chars failed: ") + std::strerror(errno));
    }
    return n;
}

ssize_t uart_port::write_uchars(const unsigned char* buf, std::size_t len)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_write_error("write_uchars: port is not open");
    }
    if (buf == nullptr || len == 0)
    {
        return 0;
    }
    ssize_t n = ::write(fd_, buf, len);
    if (n < 0)
    {
        throw uart_write_error(std::string("write_uchars failed: ") + std::strerror(errno));
    }
    return n;
}

ssize_t uart_port::write_string(const std::string& str)
{
    return write_chars(str.data(), str.size());
}

ssize_t uart_port::write_bytes(const std::vector<uint8_t>& buf)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw uart_write_error("write_bytes: port is not open");
    }
    if (buf.empty())
    {
        return 0;
    }
    ssize_t n = ::write(fd_, buf.data(), buf.size());
    if (n < 0)
    {
        throw uart_write_error(std::string("write_bytes failed: ") + std::strerror(errno));
    }
    return n;
}

// ============================================================
//  fd accessor – for poll()/epoll()/select() by external components
// ============================================================

int uart_port::get_fd() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fd_;
}

// ============================================================
//  Private helpers
// ============================================================

void uart_port::apply_termios_config()
{
    struct termios tty;
    if (::tcgetattr(fd_, &tty) < 0)
    {
        throw uart_config_error(
            std::string("tcgetattr failed: ") + std::strerror(errno));
    }

    // -------------------------------------------------------
    // Baud rate
    // -------------------------------------------------------
    speed_t speed = baud_to_speed(config_.baud);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);

    // -------------------------------------------------------
    // Raw mode baseline (disables canonical, echo, signals …)
    // -------------------------------------------------------
    ::cfmakeraw(&tty);

    // -------------------------------------------------------
    // Data bits
    // -------------------------------------------------------
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    switch (config_.data)
    {
        case data_bits::FIVE:  tty.c_cflag |= CS5; break;
        case data_bits::SIX:   tty.c_cflag |= CS6; break;
        case data_bits::SEVEN: tty.c_cflag |= CS7; break;
        case data_bits::EIGHT: tty.c_cflag |= CS8; break;
    }

    // -------------------------------------------------------
    // Parity
    // -------------------------------------------------------
    switch (config_.par)
    {
        case parity::NONE:
            tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
            break;
        case parity::ODD:
            tty.c_cflag |= PARENB | PARODD;
            break;
        case parity::EVEN:
            tty.c_cflag |= PARENB;
            tty.c_cflag &= static_cast<tcflag_t>(~PARODD);
            break;
    }

    // -------------------------------------------------------
    // Stop bits
    // -------------------------------------------------------
    if (config_.stop == stop_bits::TWO)
    {
        tty.c_cflag |= CSTOPB;
    }
    else
    {
        tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    }

    // -------------------------------------------------------
    // Flow control
    // -------------------------------------------------------
    switch (config_.flow)
    {
        case flow_control::NONE:
            tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
            tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
            break;
        case flow_control::HARDWARE:
            tty.c_cflag |= CRTSCTS;
            tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
            break;
        case flow_control::SOFTWARE:
            tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
            tty.c_iflag |= IXON | IXOFF;
            break;
    }

    // -------------------------------------------------------
    // Enable receiver and ignore modem control lines
    // -------------------------------------------------------
    tty.c_cflag |= CREAD | CLOCAL;

    // -------------------------------------------------------
    // Read timeout (VMIN / VTIME)
    //   read_timeout_ms == 0 → blocking: block until ≥1 byte arrives
    //   read_timeout_ms >  0 → timed:    return after timeout whether
    //                          or not any bytes arrived (VTIME in
    //                          tenths-of-a-second, rounded up)
    // -------------------------------------------------------
    if (config_.read_timeout_ms == 0)
    {
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 0;
    }
    else
    {
        tty.c_cc[VMIN]  = 0;
        // VTIME unit is 0.1 s – round up to ensure the requested timeout
        tty.c_cc[VTIME] = static_cast<cc_t>((config_.read_timeout_ms + 99) / 100);
    }

    if (::tcsetattr(fd_, TCSANOW, &tty) < 0)
    {
        throw uart_config_error(
            std::string("tcsetattr failed: ") + std::strerror(errno));
    }

    // Flush any garbage that may have accumulated before we applied settings
    ::tcflush(fd_, TCIOFLUSH);
}

speed_t uart_port::baud_to_speed(baud_rate baud) const
{
    switch (baud)
    {
        case baud_rate::B_50:       return B50;
        case baud_rate::B_75:       return B75;
        case baud_rate::B_110:      return B110;
        case baud_rate::B_134:      return B134;
        case baud_rate::B_150:      return B150;
        case baud_rate::B_200:      return B200;
        case baud_rate::B_300:      return B300;
        case baud_rate::B_600:      return B600;
        case baud_rate::B_1200:     return B1200;
        case baud_rate::B_1800:     return B1800;
        case baud_rate::B_2400:     return B2400;
        case baud_rate::B_4800:     return B4800;
        case baud_rate::B_9600:     return B9600;
        case baud_rate::B_19200:    return B19200;
        case baud_rate::B_38400:    return B38400;
        case baud_rate::B_57600:    return B57600;
        case baud_rate::B_115200:   return B115200;
        case baud_rate::B_230400:   return B230400;
#ifdef B460800
        case baud_rate::B_460800:   return B460800;
#endif
#ifdef B921600
        case baud_rate::B_921600:   return B921600;
#endif
#ifdef B1000000
        case baud_rate::B_1000000:  return B1000000;
#endif
#ifdef B1152000
        case baud_rate::B_1152000:  return B1152000;
#endif
#ifdef B1500000
        case baud_rate::B_1500000:  return B1500000;
#endif
#ifdef B2000000
        case baud_rate::B_2000000:  return B2000000;
#endif
#ifdef B2500000
        case baud_rate::B_2500000:  return B2500000;
#endif
#ifdef B3000000
        case baud_rate::B_3000000:  return B3000000;
#endif
#ifdef B3500000
        case baud_rate::B_3500000:  return B3500000;
#endif
#ifdef B4000000
        case baud_rate::B_4000000:  return B4000000;
#endif
        default:
            throw uart_config_error("Unsupported baud rate on this platform");
    }
}

} // namespace io
