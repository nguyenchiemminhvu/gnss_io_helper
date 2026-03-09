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
 * file_port.cpp
 * ============================================================
 * Full POSIX file implementation of file_port.
 *
 * Replay mode  (file_config::writable == false):
 *   Opens O_RDONLY.  Reads sequentially; wraps to BOF on EOF when
 *   file_config::loop == true.  write_bytes() throws file_open_error.
 *
 * Recording mode (file_config::writable == true):
 *   Opens O_RDWR | O_CREAT | O_TRUNC.  Writes append at current position.
 */

#include "file_port.h"
#include "file_exception.h"

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <cerrno>
#include <libgen.h>   // ::dirname

namespace io
{

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

file_port::file_port()
    : fd_(-1)
    , config_()
{
}

file_port::~file_port()
{
    close();
}

// ---------------------------------------------------------------------------
// Lifecycle – public (mutex-acquiring)
// ---------------------------------------------------------------------------

bool file_port::open(const i_io_config& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ >= 0)
    {
        close_locked();
    }
    return open_locked(config);
}

void file_port::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
}

bool file_port::is_open() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fd_ >= 0;
}

const i_io_config& file_port::get_config() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

bool file_port::flush()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        return false;
    }
    return ::fsync(fd_) == 0;
}

bool file_port::drain()
{
    return flush();
}

bool file_port::reconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
    return open_locked(config_);
}

bool file_port::reconfigure(const i_io_config& new_config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
    return open_locked(new_config);
}

int file_port::get_fd() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fd_;
}

// ---------------------------------------------------------------------------
// i_io_reader
// ---------------------------------------------------------------------------

ssize_t file_port::read_byte(char& c)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw file_read_error("file_port::read_byte: port not open");
    }

    ssize_t n = read_locked(&c, 1u);
    if (n < 0)
    {
        throw file_read_error(std::string("file_port::read_byte: ") + std::strerror(errno));
    }
    return n;
}

ssize_t file_port::read_chars(char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw file_read_error("file_port::read_chars: port not open");
    }

    ssize_t n = read_locked(buf, len);
    if (n < 0)
    {
        throw file_read_error(std::string("file_port::read_chars: ") + std::strerror(errno));
    }
    return n;
}

ssize_t file_port::read_uchars(unsigned char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw file_read_error("file_port::read_uchars: port not open");
    }

    ssize_t n = read_locked(buf, len);
    if (n < 0)
    {
        throw file_read_error(std::string("file_port::read_uchars: ") + std::strerror(errno));
    }
    return n;
}

ssize_t file_port::read_string(std::string& str, std::size_t max_len)
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

ssize_t file_port::read_bytes(std::vector<uint8_t>& buf, std::size_t max_len)
{
    if (max_len == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw file_read_error("file_port::read_bytes: port not open");
    }

    buf.resize(max_len);
    ssize_t n = read_locked(buf.data(), max_len);
    if (n < 0)
    {
        throw file_read_error(std::string("file_port::read_bytes: ") + std::strerror(errno));
    }
    buf.resize(static_cast<std::size_t>(n));
    return n;
}

int file_port::available() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        return -1;
    }

    // Current read position.
    off_t cur = ::lseek(fd_, 0, SEEK_CUR);
    if (cur < 0)
    {
        return -1;
    }

    // File size via stat.
    struct stat st{};
    if (::fstat(fd_, &st) < 0)
    {
        return -1;
    }

    off_t remaining = st.st_size - cur;
    if (remaining > 0)
    {
        return static_cast<int>(remaining);
    }

    // In loop mode the next read will wrap back to BOF, so report the full
    // file size as available rather than returning 0 (which causes the
    // worker to spin endlessly on POLLIN without ever calling read).
    if (config_.loop && st.st_size > 0)
    {
        return static_cast<int>(st.st_size);
    }

    return 0;
}

// ---------------------------------------------------------------------------
// i_io_writer
// ---------------------------------------------------------------------------

ssize_t file_port::write_byte(char c)
{
    return write_bytes(std::vector<uint8_t>{static_cast<uint8_t>(c)});
}

ssize_t file_port::write_chars(const char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }
    const uint8_t* p = reinterpret_cast<const uint8_t*>(buf);
    return write_bytes(std::vector<uint8_t>(p, p + len));
}

ssize_t file_port::write_uchars(const unsigned char* buf, std::size_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }
    const uint8_t* p = reinterpret_cast<const uint8_t*>(buf);
    return write_bytes(std::vector<uint8_t>(p, p + len));
}

ssize_t file_port::write_string(const std::string& str)
{
    const uint8_t* p = reinterpret_cast<const uint8_t*>(str.data());
    return write_bytes(std::vector<uint8_t>(p, p + str.size()));
}

ssize_t file_port::write_bytes(const std::vector<uint8_t>& buf)
{
    if (buf.empty())
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0)
    {
        throw file_open_error("file_port::write_bytes: port not open");
    }

    if (!config_.writable)
    {
        throw file_open_error(
            "file_port::write_bytes: file opened in read-only (replay) mode");
    }

    const uint8_t* ptr = buf.data();
    std::size_t    remaining = buf.size();
    while (remaining > 0)
    {
        ssize_t n = ::write(fd_, ptr, remaining);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            throw file_open_error(
                std::string("file_port::write_bytes: write failed: ")
                + std::strerror(errno));
        }
        ptr       += static_cast<std::size_t>(n);
        remaining -= static_cast<std::size_t>(n);
    }
    return static_cast<ssize_t>(buf.size());
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool file_port::open_locked(const i_io_config& config)
{
    if (config.channel_type() != io_channel_type::file)
    {
        throw file_open_error("file_port::open: config is not file_config");
    }

    const auto& cfg = static_cast<const file_config&>(config);

    int flags = cfg.writable
                ? (O_RDWR | O_CREAT | O_TRUNC)
                : O_RDONLY;

    // When creating a new file, ensure parent directories exist.
    if (cfg.writable)
    {
        // ::dirname may modify its argument, so work on a copy.
        std::string path_copy = cfg.file_path;
        char* dir = ::dirname(path_copy.data());
        if (dir && dir[0] != '\0')
        {
            // Recursively mkdir the parent directory (like mkdir -p).
            std::string dir_str(dir);
            for (std::size_t pos = 1; pos <= dir_str.size(); ++pos)
            {
                if (pos == dir_str.size() || dir_str[pos] == '/')
                {
                    std::string partial = dir_str.substr(0, pos);
                    if (::mkdir(partial.c_str(), 0755) < 0 && errno != EEXIST)
                    {
                        throw file_open_error(
                            "file_port::open: cannot create directory " + partial
                            + ": " + std::strerror(errno));
                    }
                }
            }
        }
    }

    // mode bits used only when O_CREAT is set.
    int new_fd = ::open(cfg.file_path.c_str(), flags, 0644);
    if (new_fd < 0)
    {
        throw file_open_error(
            "file_port::open: cannot open " + cfg.file_path
            + ": " + std::strerror(errno));
    }

    fd_     = new_fd;
    config_ = cfg;
    return true;
}

void file_port::close_locked()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

ssize_t file_port::read_locked(void* buf, std::size_t len)
{
    uint8_t* ptr = static_cast<uint8_t*>(buf);
    std::size_t total = 0;

    while (total < len)
    {
        ssize_t n = ::read(fd_, ptr + total, len - total);

        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return -1;                  // propagate error to caller
        }

        if (n == 0)
        {
            // EOF
            if (config_.loop && total == 0)
            {
                // Loop: seek back to beginning and retry.
                if (::lseek(fd_, 0, SEEK_SET) < 0)
                {
                    return -1;
                }
                continue;
            }
            break;                      // non-loop or partial read: stop here
        }

        total += static_cast<std::size_t>(n);
    }

    // Apply inter-read throttle if configured.
    if (config_.throttle_us > 0 && total > 0)
    {
        ::usleep(static_cast<useconds_t>(config_.throttle_us));
    }

    return static_cast<ssize_t>(total);
}

} // namespace io
