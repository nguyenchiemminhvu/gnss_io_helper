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
 * file_exception.h
 * ============================================================
 * Record-file-specific exception subtypes deriving from the base
 * io_helper exception hierarchy (io_exception.h).
 */

#ifndef FILE_EXCEPTION_H
#define FILE_EXCEPTION_H

#include "../io_exception.h"

namespace io
{

class file_open_error : public io_open_error
{
public:
    explicit file_open_error(const std::string& msg)
        : io_open_error(msg) {}
};

class file_read_error : public io_read_error
{
public:
    explicit file_read_error(const std::string& msg)
        : io_read_error(msg) {}
};

} // namespace io

#endif // FILE_EXCEPTION_H
