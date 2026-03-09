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
 * i_io_reader.h
 * ============================================================
 * Interface Segregation (SOLID – ISP): read-only contract for an
 * I/O data source.
 *
 * Consumers that only need to receive data (e.g. a UBX frame
 * decoder) depend on this interface rather than the full
 * i_io_port, keeping coupling minimal.
 *
 * All read methods return the number of bytes actually read, or
 * throw io_read_error / io_timeout_error on failure.
 * A return value of 0 means the timeout elapsed without data
 * (non-error condition when read_timeout_ms > 0).
 */

#ifndef I_IO_READER_H
#define I_IO_READER_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <sys/types.h>   // ssize_t

namespace io
{

class i_io_reader
{
public:
    virtual ~i_io_reader() = default;

    /**
     * @brief Read exactly one byte from the channel.
     *
     * @param[out] c  Receives the byte that was read.
     * @return 1 on success, 0 on timeout (when configured).
     * @throws io_read_error on I/O error.
     */
    virtual ssize_t read_byte(char& c) = 0;

    /**
     * @brief Read up to `len` bytes into a raw char buffer.
     *
     * The caller owns the buffer and must ensure it is at least
     * `len` bytes long.
     *
     * @param[out] buf  Destination buffer.
     * @param[in]  len  Maximum bytes to read.
     * @return Number of bytes placed in buf (0 on timeout).
     * @throws io_read_error on I/O error.
     */
    virtual ssize_t read_chars(char* buf, std::size_t len) = 0;

    /**
     * @brief Read up to `len` bytes into a raw unsigned char buffer.
     *
     * Same semantics as read_chars(), but for consumers that prefer unsigned
     * byte buffers (e.g. UBX frame decoder).
     *
     * @param[out] buf  Destination buffer.
     * @param[in]  len  Maximum bytes to read.
     * @return Number of bytes placed in buf (0 on timeout).
     * @throws io_read_error on I/O error.
     */
    virtual ssize_t read_uchars(unsigned char* buf, std::size_t len) = 0;

    /**
     * @brief Read up to `max_len` bytes and place them in `str`.
     *
     * The string is resized to exactly the number of bytes read.
     * Useful for NMEA sentence reception where the data is
     * printable ASCII.
     *
     * @param[out] str      Receives the data (existing contents replaced).
     * @param[in]  max_len  Maximum bytes to read.
     * @return Number of bytes read (0 on timeout).
     * @throws io_read_error on I/O error.
     */
    virtual ssize_t read_string(std::string& str, std::size_t max_len) = 0;

    /**
     * @brief Read up to `max_len` bytes into a byte vector.
     *
     * The vector is resized to exactly the number of bytes read.
     * Preferred for binary UBX protocol data.
     *
     * @param[out] buf      Receives the raw bytes (existing contents replaced).
     * @param[in]  max_len  Maximum bytes to read.
     * @return Number of bytes read (0 on timeout).
     * @throws io_read_error on I/O error.
     */
    virtual ssize_t read_bytes(std::vector<uint8_t>& buf, std::size_t max_len) = 0;

    /**
     * @brief Query how many bytes are waiting in the receive buffer.
     *
     * Uses ioctl(FIONREAD) for UART; analogous mechanism for other channels.
     * Returns -1 if the channel is not open or the query fails.
     *
     * @return Number of bytes available to read without blocking, or -1.
     */
    virtual int available() const = 0;
};

} // namespace io

#endif // I_IO_READER_H
