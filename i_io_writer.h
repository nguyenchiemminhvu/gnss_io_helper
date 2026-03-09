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
 * i_io_writer.h
 * ============================================================
 * Interface Segregation (SOLID – ISP): write-only contract for an
 * I/O data sink.
 *
 * Consumers that only need to send data (e.g. a UBX command
 * builder) depend on this narrow interface rather than the full
 * i_io_port contract.
 *
 * All write methods return the number of bytes actually written,
 * or throw io_write_error on failure.  A short write (return
 * value < requested) is not an error; it is the caller's
 * responsibility to re-issue the remainder if needed.
 */

#ifndef I_IO_WRITER_H
#define I_IO_WRITER_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <sys/types.h>   // ssize_t

namespace io
{

class i_io_writer
{
public:
    virtual ~i_io_writer() = default;

    /**
     * @brief Write a single byte to the channel.
     *
     * @param[in] c  Byte to transmit.
     * @return 1 on success.
     * @throws io_write_error on I/O error.
     */
    virtual ssize_t write_byte(char c) = 0;

    /**
     * @brief Write up to `len` bytes from a raw char buffer.
     *
     * @param[in] buf  Source buffer (must not be null when len > 0).
     * @param[in] len  Number of bytes to write.
     * @return Number of bytes actually written (may be < len).
     * @throws io_write_error on I/O error.
     */
    virtual ssize_t write_chars(const char* buf, std::size_t len) = 0;

    /**
     * @brief Write up to `len` bytes from a raw unsigned char buffer.
     *
     * @param[in] buf  Source buffer (must not be null when len > 0).
     * @param[in] len  Number of bytes to write.
     * @return Number of bytes actually written (may be < len).
     * @throws io_write_error on I/O error.
     */
    virtual ssize_t write_uchars(const unsigned char* buf, std::size_t len) = 0;

    /**
     * @brief Write the contents of a std::string to the channel.
     *
     * Convenient for sending NMEA sentences or ASCII AT-style
     * commands.  Null terminator is NOT transmitted.
     *
     * @param[in] str  String to transmit.
     * @return Number of bytes actually written.
     * @throws io_write_error on I/O error.
     */
    virtual ssize_t write_string(const std::string& str) = 0;

    /**
     * @brief Write the contents of a byte vector to the channel.
     *
     * Preferred for binary UBX protocol frames. The entire
     * vector contents are written (or as many bytes as the
     * underlying channel accepted in one call).
     *
     * @param[in] buf  Bytes to transmit.
     * @return Number of bytes actually written.
     * @throws io_write_error on I/O error.
     */
    virtual ssize_t write_bytes(const std::vector<uint8_t>& buf) = 0;
};

} // namespace io

#endif // I_IO_WRITER_H
