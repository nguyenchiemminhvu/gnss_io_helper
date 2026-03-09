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
 * io_exception.h
 * ============================================================
 * Base exception hierarchy for the io_helper library.
 *
 * Protocol-specific exception subtypes (uart_open_error, i2c_read_error,
 * etc.) are defined in their respective sub-folder headers and derive
 * from the corresponding base class here.
 *
 * All exceptions derive from io_exception, which itself derives from
 * std::runtime_error, so callers can catch at any granularity:
 *
 *   catch (const io::uart_open_error& e)  { ... }  // UART-specific
 *   catch (const io::io_open_error& e)    { ... }  // any open error
 *   catch (const io::io_exception& e)     { ... }  // any io_helper error
 *   catch (const std::runtime_error& e)   { ... }  // any runtime error
 *
 * Hierarchy:
 *   std::runtime_error
 *   └── io_exception
 *       ├── io_open_error    – open() / resource acquisition failed
 *       ├── io_config_error  – protocol configuration failed
 *       ├── io_read_error    – read operation failed
 *       ├── io_write_error   – write operation failed
 *       └── io_timeout_error – operation timed out
 */

#ifndef IO_EXCEPTION_H
#define IO_EXCEPTION_H

#include <stdexcept>
#include <string>

namespace io
{

/**
 * @brief Base class for all io_helper exceptions.
 */
class io_exception : public std::runtime_error
{
public:
    explicit io_exception(const std::string& msg)
        : std::runtime_error(msg) {}
};

/**
 * @brief Thrown when a channel resource cannot be opened.
 *
 * Common causes: device path does not exist, permission denied,
 * device busy (another process holds the port).
 */
class io_open_error : public io_exception
{
public:
    explicit io_open_error(const std::string& msg)
        : io_exception(msg) {}
};

/**
 * @brief Thrown when applying channel configuration fails.
 *
 * Common causes: unsupported baud rate for the hardware driver,
 * invalid combination of channel parameters.
 */
class io_config_error : public io_exception
{
public:
    explicit io_config_error(const std::string& msg)
        : io_exception(msg) {}
};

/**
 * @brief Thrown when a read operation fails.
 *
 * Does NOT cover timeout (zero-byte return); only covers negative
 * return values from the underlying read call.
 */
class io_read_error : public io_exception
{
public:
    explicit io_read_error(const std::string& msg)
        : io_exception(msg) {}
};

/**
 * @brief Thrown when a write operation fails.
 */
class io_write_error : public io_exception
{
public:
    explicit io_write_error(const std::string& msg)
        : io_exception(msg) {}
};

/**
 * @brief Thrown when an operation exceeds its configured timeout.
 *
 * Raised by higher-level helpers that implement explicit deadline
 * semantics on top of non-blocking / VTIME termios mechanisms.
 */
class io_timeout_error : public io_exception
{
public:
    explicit io_timeout_error(const std::string& msg)
        : io_exception(msg) {}
};

} // namespace io

#endif // IO_EXCEPTION_H
