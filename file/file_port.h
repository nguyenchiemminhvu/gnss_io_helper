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
 * file_port.h
 * ============================================================
 * Concrete POSIX file implementation of i_io_port for binary
 * GNSS data recording and replay.
 *
 * Replay mode  (file_config::writable == false):
 *  • Opens the file O_RDONLY.
 *  • read_bytes() reads sequentially; when EOF is hit and
 *    file_config::loop == true the file position wraps to the start.
 *  • available() returns remaining bytes from the current position
 *    to EOF.
 *  • write_bytes() throws file_open_error (read-only channel).
 *
 * Recording mode (file_config::writable == true):
 *  • Opens the file O_RDWR | O_CREAT | O_TRUNC.
 *  • write_bytes() appends data at the current write position.
 *  • read_bytes() reads from the current read position.
 *
 * get_fd() returns the underlying POSIX fd so callers can use
 * poll()/select() on the file (useful for pipe-backed sources
 * or named-pipe playback setups).
 *
 * Thread-safety: every public method acquires an internal mutex.
 */

#ifndef FILE_PORT_H
#define FILE_PORT_H

#include "../i_io_port.h"
#include "file_config.h"

#include <mutex>

namespace io
{

class file_port final : public i_io_port
{
public:
    file_port();
    ~file_port() override;

    file_port(const file_port&)            = delete;
    file_port& operator=(const file_port&) = delete;

    // -----------------------------------------------------------
    // i_io_port – Lifecycle
    // -----------------------------------------------------------

    /**
     * @brief Open the recording/replay file.
     *
     * @throws file_open_error   if `config` is not a file_config.
     * @throws file_open_error   if the file cannot be opened/created.
     */
    bool open(const i_io_config& config) override;
    void close() override;
    bool is_open() const override;
    const i_io_config& get_config() const override;

    /**
     * @brief Flushes kernel write buffers via fsync().
     * @return true on success, false if the file is not open.
     */
    bool flush() override;

    /**
     * @brief Synonym for flush() – ensures all written data reaches storage.
     * @return true on success, false if the file is not open.
     */
    bool drain() override;

    bool reconnect() override;
    bool reconfigure(const i_io_config& new_config) override;

    /**
     * @brief Return the underlying POSIX fd.
     *
     * Suitable for use with poll()/select() when the file is a named pipe
     * or special device node that signals POLLIN on data availability.
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
     * @brief Return remaining bytes from the current position to EOF.
     *
     * Uses fstat() + lseek(SEEK_CUR).  Returns -1 on error or when
     * the file is not open.
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
    int                 fd_;      ///< POSIX file descriptor; -1 when closed
    file_config         config_;  ///< Last successfully applied configuration
    mutable std::mutex  mutex_;   ///< Guards all fd_ accesses

    bool open_locked(const i_io_config& config);
    void close_locked();

    /**
     * @brief Read exactly `len` bytes, handling EOF + loop wrap.
     *
     * Must be called with mutex_ already held.
     * On EOF with loop enabled, seeks to file start and continues.
     * Returns the number of bytes actually placed in `buf`.
     */
    ssize_t read_locked(void* buf, std::size_t len);
};

} // namespace io

#endif // FILE_PORT_H
