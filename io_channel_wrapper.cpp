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
 * io_channel_wrapper.cpp
 * ============================================================
 * Implementation of the high-level io_channel_wrapper class.
 */

#include "io_channel_wrapper.h"
#include "io_exception.h"

#include <stdexcept>

namespace io
{

// ============================================================
//  Constructor / Destructor
// ============================================================

io_channel_wrapper::io_channel_wrapper(std::shared_ptr<i_io_port> port)
    : port_(port)
{
}

io_channel_wrapper::~io_channel_wrapper()
{
}

// ============================================================
//  Lifecycle
// ============================================================

bool io_channel_wrapper::initialize(const i_io_config& config)
{
    if (auto port = port_.lock())
    {
        return port->open(config);
    }
    return false;
}

void io_channel_wrapper::shutdown()
{
    if (auto port = port_.lock())
    {
        if (port->is_open())
        {
            port->close();
        }
    }
}

bool io_channel_wrapper::reconnect()
{
    if (auto port = port_.lock())
    {
        return port->reconnect();
    }
    return false;
}

bool io_channel_wrapper::reconfigure(const i_io_config& new_config)
{
    if (auto port = port_.lock())
    {
        return port->reconfigure(new_config);
    }
    return false;
}

// ============================================================
//  Status
// ============================================================

bool io_channel_wrapper::is_ready() const
{
    if (auto port = port_.lock())
    {
        return port->is_open();
    }
    return false;
}

const i_io_config& io_channel_wrapper::get_config() const
{
    if (auto port = port_.lock())
    {
        return port->get_config();
    }
    throw std::runtime_error("io_channel_wrapper: port is not available");
}

// ============================================================
//  fd accessor
// ============================================================

int io_channel_wrapper::get_fd() const noexcept
{
    if (auto port = port_.lock())
    {
        return port->get_fd();
    }
    return -1;
}

// ============================================================
//  Read operations
// ============================================================

ssize_t io_channel_wrapper::read_byte(char& c)
{
    if (auto port = port_.lock())
    {
        return port->read_byte(c);
    }
    return -1;
}

ssize_t io_channel_wrapper::read_chars(char* buf, std::size_t len)
{
    if (auto port = port_.lock())
    {
        return port->read_chars(buf, len);
    }
    return -1;
}

ssize_t io_channel_wrapper::read_uchars(unsigned char* buf, std::size_t len)
{
    if (auto port = port_.lock())
    {
        return port->read_uchars(buf, len);
    }
    return -1;
}

ssize_t io_channel_wrapper::read_string(std::string& str, std::size_t max_len)
{
    if (auto port = port_.lock())
    {
        return port->read_string(str, max_len);
    }
    return -1;
}

ssize_t io_channel_wrapper::read_bytes(std::vector<uint8_t>& buf, std::size_t max_len)
{
    if (auto port = port_.lock())
    {
        return port->read_bytes(buf, max_len);
    }
    return -1;
}

int io_channel_wrapper::available() const
{
    if (auto port = port_.lock())
    {
        return port->available();
    }
    return -1;
}

// ============================================================
//  Write operations
// ============================================================

ssize_t io_channel_wrapper::write_byte(char c)
{
    if (auto port = port_.lock())
    {
        return port->write_byte(c);
    }
    return -1;
}

ssize_t io_channel_wrapper::write_chars(const char* buf, std::size_t len)
{
    if (auto port = port_.lock())
    {
        return port->write_chars(buf, len);
    }
    return -1;
}

ssize_t io_channel_wrapper::write_uchars(const unsigned char* buf, std::size_t len)
{
    if (auto port = port_.lock())
    {
        return port->write_uchars(buf, len);
    }
    return -1;
}

ssize_t io_channel_wrapper::write_string(const std::string& str)
{
    if (auto port = port_.lock())
    {
        return port->write_string(str);
    }
    return -1;
}

ssize_t io_channel_wrapper::write_bytes(const std::vector<uint8_t>& buf)
{
    if (auto port = port_.lock())
    {
        return port->write_bytes(buf);
    }
    return -1;
}

// ============================================================
//  Buffer management
// ============================================================

bool io_channel_wrapper::flush()
{
    if (auto port = port_.lock())
    {
        return port->flush();
    }
    return false;
}

bool io_channel_wrapper::drain()
{
    if (auto port = port_.lock())
    {
        return port->drain();
    }
    return false;
}

} // namespace io
