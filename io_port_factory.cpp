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
 * io_port_factory.cpp
 * ============================================================
 * Implementation of io_port_factory::create().
 *
 * This is the only translation unit that includes concrete sub-folder
 * headers (uart/uart_port.h, i2c/i2c_port.h, file/file_port.h).
 * Adding a new protocol requires:
 *   1. A new spi/ (or tcp/, etc.) sub-folder with its own config/port files.
 *   2. A new case in the switch below.
 *   Zero other files need to change.
 */

#include "io_port_factory.h"
#include "uart/uart_port.h"
#include "i2c/i2c_port.h"
#include "file/file_port.h"

#include <stdexcept>

namespace io
{

std::shared_ptr<i_io_port> io_port_factory::create(const i_io_config& config)
{
    switch (config.channel_type())
    {
        case io_channel_type::uart:
        {
            return std::make_shared<uart_port>();
        }

        case io_channel_type::i2c:
        {
            return std::make_shared<i2c_port>();
        }
        case io_channel_type::file:
        {
            return std::make_shared<file_port>();
        }

        default:
        {
            throw std::invalid_argument(
                "io_port_factory: unsupported or unimplemented channel type");
        }
    }
}

} // namespace io
