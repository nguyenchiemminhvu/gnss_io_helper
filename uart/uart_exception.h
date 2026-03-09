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
 * uart_exception.h
 * ============================================================
 * UART-specific exception subtypes deriving from the base
 * io_helper exception hierarchy (io_exception.h).
 *
 * Callers may catch at any level of granularity:
 *
 *   catch (const io::uart_open_error& e)   { ... }  // UART open only
 *   catch (const io::io_open_error& e)     { ... }  // any open error
 *   catch (const io::io_exception& e)      { ... }  // any io_helper error
 *   catch (const std::runtime_error& e)    { ... }  // any runtime error
 */

#ifndef UART_EXCEPTION_H
#define UART_EXCEPTION_H

#include "../io_exception.h"

namespace io
{

/**
 * @brief Thrown when the UART device node cannot be opened.
 *
 * Common causes: device path does not exist, permission denied,
 * device busy (another process holds the port).
 */
class uart_open_error : public io_open_error
{
public:
    explicit uart_open_error(const std::string& msg)
        : io_open_error(msg) {}
};

/**
 * @brief Thrown when applying termios settings to the UART fails.
 *
 * Common causes: unsupported baud rate for the hardware driver,
 * invalid combination of data/parity/stop parameters, or the
 * supplied i_io_config is not a uart_config.
 */
class uart_config_error : public io_config_error
{
public:
    explicit uart_config_error(const std::string& msg)
        : io_config_error(msg) {}
};

/**
 * @brief Thrown when a UART read operation fails.
 *
 * Does NOT cover timeout (zero-byte return); only covers negative
 * return values from read() / ioctl(FIONREAD).
 */
class uart_read_error : public io_read_error
{
public:
    explicit uart_read_error(const std::string& msg)
        : io_read_error(msg) {}
};

/**
 * @brief Thrown when a UART write operation fails.
 */
class uart_write_error : public io_write_error
{
public:
    explicit uart_write_error(const std::string& msg)
        : io_write_error(msg) {}
};

} // namespace io

#endif // UART_EXCEPTION_H
