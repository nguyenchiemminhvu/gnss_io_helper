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
 * i2c_port.cpp
 * ============================================================
 * Full Linux i2c-dev implementation of i2c_port.
 *
 * Depends on:
 *   /usr/include/linux/i2c-dev.h  (I2C_RDWR, I2C_SLAVE, i2c_msg,
 *                                   i2c_rdwr_ioctl_data)
 *   /usr/include/linux/i2c.h      (I2C_M_RD flag)
 */

#include "i2c_port.h"
#include "i2c_exception.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <cerrno>
#include <cstdio>

namespace io
{

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

i2c_port::i2c_port()
    : fd_(-1)
    , config_()
{
}

i2c_port::~i2c_port()
{
    close();
}

// ---------------------------------------------------------------------------
// Lifecycle – public (mutex-acquiring)
// ---------------------------------------------------------------------------

bool i2c_port::open(const i_io_config& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ >= 0)
    {
        close_locked();
    }
    return open_locked(config);
}

void i2c_port::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
}

bool i2c_port::is_open() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fd_ >= 0;
}

const i_io_config& i2c_port::get_config() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

bool i2c_port::flush()
{
    // I2C bus transactions are synchronous – nothing to flush.
    return true;
}

bool i2c_port::drain()
{
    // I2C bus transactions are synchronous – nothing to drain.
    return true;
}

bool i2c_port::reconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
    return open_locked(config_);
}

bool i2c_port::reconfigure(const i_io_config& new_config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
    return open_locked(new_config);
}

int i2c_port::get_fd() const noexcept
{
    // The i2c-dev fd is not suitable for poll()/select().
    return -1;
}

// ---------------------------------------------------------------------------
// i_io_reader
// ---------------------------------------------------------------------------

ssize_t i2c_port::read_byte(char& c)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw i2c_read_error("i2c_port::read_byte: port not open");
    }

    const uint8_t reg = I2C_DDC_REG_DATA_STREAM;
    uint8_t byte_val  = 0;
    ssize_t n = rdwr_locked(&reg, 1u, &byte_val, 1u);
    if (n < 0)
    {
        throw i2c_read_error("i2c_port::read_byte: rdwr failed");
    }
    c = static_cast<char>(byte_val);
    return n;
}

ssize_t i2c_port::read_chars(char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw i2c_read_error("i2c_port::read_chars: port not open");
    }

    const uint8_t reg = I2C_DDC_REG_DATA_STREAM;
    ssize_t n = rdwr_locked(&reg, 1u,
                            reinterpret_cast<uint8_t*>(buf), len);
    if (n < 0)
    {
        throw i2c_read_error("i2c_port::read_chars: rdwr failed");
    }
    return n;
}

ssize_t i2c_port::read_uchars(unsigned char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw i2c_read_error("i2c_port::read_uchars: port not open");
    }

    const uint8_t reg = I2C_DDC_REG_DATA_STREAM;
    ssize_t n = rdwr_locked(&reg, 1u,
                            reinterpret_cast<uint8_t*>(buf), len);
    if (n < 0)
    {
        throw i2c_read_error("i2c_port::read_uchars: rdwr failed");
    }
    return n;
}

ssize_t i2c_port::read_string(std::string& str, std::size_t max_len)
{
    if (max_len == 0)
    {
        return 0;
    }

    std::vector<uint8_t> tmp;
    ssize_t n = read_bytes(tmp, max_len);
    if (n > 0)
    {
        str.assign(reinterpret_cast<const char*>(tmp.data()),
                   static_cast<std::size_t>(n));
    }
    return n;
}

ssize_t i2c_port::read_bytes(std::vector<uint8_t>& buf, std::size_t max_len)
{
    if (max_len == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw i2c_read_error("i2c_port::read_bytes: port not open");
    }

    buf.resize(max_len);
    const uint8_t reg = I2C_DDC_REG_DATA_STREAM;
    ssize_t n = rdwr_locked(&reg, 1u, buf.data(), max_len);
    if (n < 0)
    {
        throw i2c_read_error("i2c_port::read_bytes: rdwr failed");
    }
    buf.resize(static_cast<std::size_t>(n));
    return n;
}

int i2c_port::available() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        return -1;
    }

    // DDC protocol: write register 0xFD, read 2 bytes (big-endian count).
    const uint8_t reg = I2C_DDC_REG_BYTES_AVAIL_MSB;
    uint8_t avail_buf[2] = {0, 0};
    ssize_t n = rdwr_locked(&reg, 1u, avail_buf, 2u);
    if (n != 2)
    {
        return -1;
    }

    return static_cast<int>((static_cast<uint16_t>(avail_buf[0]) << 8u)
                            | static_cast<uint16_t>(avail_buf[1]));
}

// ---------------------------------------------------------------------------
// i_io_writer
// ---------------------------------------------------------------------------

ssize_t i2c_port::write_byte(char c)
{
    uint8_t payload = static_cast<uint8_t>(c);
    return write_bytes(std::vector<uint8_t>{payload});
}

ssize_t i2c_port::write_chars(const char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }
    const uint8_t* p = reinterpret_cast<const uint8_t*>(buf);
    return write_bytes(std::vector<uint8_t>(p, p + len));
}

ssize_t i2c_port::write_uchars(const unsigned char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }
    const uint8_t* p = reinterpret_cast<const uint8_t*>(buf);
    return write_bytes(std::vector<uint8_t>(p, p + len));
}

ssize_t i2c_port::write_string(const std::string& str)
{
    const uint8_t* p = reinterpret_cast<const uint8_t*>(str.data());
    return write_bytes(std::vector<uint8_t>(p, p + str.size()));
}

ssize_t i2c_port::write_bytes(const std::vector<uint8_t>& buf)
{
    if (buf.empty())
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw i2c_write_error("i2c_port::write_bytes: port not open");
    }

    // Prepend the DDC data-write register address (0x00) to the payload.
    std::vector<uint8_t> frame;
    frame.reserve(1u + buf.size());
    frame.push_back(I2C_DDC_REG_DATA_WRITE);
    frame.insert(frame.end(), buf.begin(), buf.end());

    struct i2c_msg msg{};
    msg.addr  = config_.address;
    msg.flags = 0;                              // write
    msg.len   = static_cast<__u16>(frame.size());
    msg.buf   = frame.data();

    struct i2c_rdwr_ioctl_data xfer{};
    xfer.msgs  = &msg;
    xfer.nmsgs = 1;

    int rc = ::ioctl(fd_, I2C_RDWR, &xfer);
    if (rc < 0)
    {
        throw i2c_write_error(
            std::string("i2c_port::write_bytes: I2C_RDWR failed: ")
            + std::strerror(errno));
    }

    return static_cast<ssize_t>(buf.size());
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool i2c_port::open_locked(const i_io_config& config)
{
    if (config.channel_type() != io_channel_type::i2c)
    {
        throw i2c_config_error("i2c_port::open: config is not i2c_config");
    }

    const auto& cfg = static_cast<const i2c_config&>(config);

    int new_fd = ::open(cfg.device_path.c_str(), O_RDWR);
    if (new_fd < 0)
    {
        throw i2c_open_error(
            "i2c_port::open: cannot open " + cfg.device_path
            + ": " + std::strerror(errno));
    }

    // Select the slave address.
    if (::ioctl(new_fd, I2C_SLAVE, static_cast<long>(cfg.address)) < 0)
    {
        ::close(new_fd);
        char addr_str[8];
        std::snprintf(addr_str, sizeof(addr_str), "%02X", cfg.address);
        throw i2c_config_error(
            std::string("i2c_port::open: I2C_SLAVE ioctl failed for address 0x")
            + addr_str + ": " + std::strerror(errno));
    }

    fd_     = new_fd;
    config_ = cfg;
    return true;
}

void i2c_port::close_locked()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

ssize_t i2c_port::rdwr_locked(const uint8_t* write_buf, std::size_t write_len,
                               uint8_t*       read_buf,  std::size_t read_len) const
{
    // Two-message combined transaction: write (register select) + read (data).
    struct i2c_msg msgs[2]{};

    // Message 0: write phase (set register pointer).
    msgs[0].addr  = config_.address;
    msgs[0].flags = 0;
    msgs[0].len   = static_cast<__u16>(write_len);
    msgs[0].buf   = const_cast<uint8_t*>(write_buf);  // kernel reads only

    // Message 1: read phase (fetch data).
    msgs[1].addr  = config_.address;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = static_cast<__u16>(read_len);
    msgs[1].buf   = read_buf;

    struct i2c_rdwr_ioctl_data xfer{};
    xfer.msgs  = msgs;
    xfer.nmsgs = 2;

    int rc = ::ioctl(fd_, I2C_RDWR, &xfer);
    if (rc < 0)
    {
        return -1;
    }

    // ioctl returns number of messages completed (2 on success).
    return static_cast<ssize_t>(read_len);
}

} // namespace io
