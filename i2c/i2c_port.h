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
 * i2c_port.h
 * ============================================================
 * Concrete Linux i2c-dev implementation of i_io_port for the
 * u-blox DDC (I2C) interface.
 *
 * Uses the standard Linux i2c-dev kernel module and the
 * I2C_RDWR ioctl for combined write-then-read transactions
 * (register pointer set + data read in a single bus transaction).
 *
 * u-blox DDC protocol:
 *  • available() – combined I2C_RDWR: write 0xFD, read 2 bytes → big-endian count
 *  • read_bytes() – combined I2C_RDWR: write 0xFF, read N bytes
 *  • write_bytes() – single write: [0x00, payload…]
 *
 * Thread-safety: every public method acquires an internal mutex.
 *
 * Note: get_fd() returns -1 – the i2c-dev fd is not suitable for
 * poll()/select() because I2C does not signal POLLIN; callers must
 * use a timed-polling loop via available().
 */

#ifndef I2C_PORT_H
#define I2C_PORT_H

#include "../i_io_port.h"
#include "i2c_config.h"

#include <mutex>

namespace io
{

class i2c_port final : public i_io_port
{
public:
    i2c_port();
    ~i2c_port() override;

    i2c_port(const i2c_port&)            = delete;
    i2c_port& operator=(const i2c_port&) = delete;

    // -----------------------------------------------------------
    // i_io_port – Lifecycle
    // -----------------------------------------------------------

    /**
     * @brief Open the i2c-dev adapter and select the slave address.
     *
     * @throws i2c_config_error if `config` is not an i2c_config.
     * @throws i2c_open_error   if the adapter node cannot be opened.
     * @throws i2c_config_error if I2C_SLAVE ioctl fails.
     */
    bool open(const i_io_config& config) override;
    void close() override;
    bool is_open() const override;
    const i_io_config& get_config() const override;

    /**
     * @brief No-op for I2C – there is no kernel Tx/Rx buffer to flush.
     * @return true always.
     */
    bool flush() override;

    /**
     * @brief No-op for I2C – bus transactions are synchronous.
     * @return true always.
     */
    bool drain() override;

    bool reconnect() override;
    bool reconfigure(const i_io_config& new_config) override;

    /**
     * @brief Returns -1 – I2C does not expose a pollable fd.
     */
    int  get_fd() const noexcept override;

    // -----------------------------------------------------------
    // i_io_reader
    // -----------------------------------------------------------
    ssize_t read_byte(char& c) override;
    ssize_t read_chars(char* buf, std::size_t len) override;
    ssize_t read_uchars(unsigned char* buf, std::size_t len) override;
    ssize_t read_string(std::string& str, std::size_t max_len) override;
    ssize_t read_bytes(std::vector<uint8_t>& buf, std::size_t max_len) override;

    /**
     * @brief Query bytes available via u-blox DDC registers 0xFD / 0xFE.
     *
     * Issues a combined I2C_RDWR transaction (write 0xFD, read 2 bytes)
     * to retrieve the 16-bit big-endian count from the chip's internal queue.
     *
     * @return Number of bytes waiting, 0 if none, or -1 on error.
     */
    int     available() const override;

    // -----------------------------------------------------------
    // i_io_writer
    // -----------------------------------------------------------
    ssize_t write_byte(char c) override;
    ssize_t write_chars(const char* buf, std::size_t len) override;
    ssize_t write_uchars(const unsigned char* buf, std::size_t len) override;
    ssize_t write_string(const std::string& str) override;
    ssize_t write_bytes(const std::vector<uint8_t>& buf) override;

private:
    int                 fd_;      ///< i2c-dev file descriptor; -1 when closed
    i2c_config          config_;  ///< Last successfully applied configuration
    mutable std::mutex  mutex_;   ///< Guards all fd_ accesses

    bool open_locked(const i_io_config& config);
    void close_locked();

    /**
     * @brief Issue a combined write-then-read I2C_RDWR transaction.
     *
     * Writes `write_buf` (len bytes) to the slave, then reads `read_len`
     * bytes back in a single bus transaction (no STOP between phases).
     * Must be called with mutex_ already held.
     *
     * @return Number of bytes read, or -1 on error.
     */
    ssize_t rdwr_locked(const uint8_t* write_buf, std::size_t write_len,
                        uint8_t*       read_buf,  std::size_t read_len) const;
};

} // namespace io

#endif // I2C_PORT_H
