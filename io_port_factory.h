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
 * io_port_factory.h
 * ============================================================
 * Factory that creates the correct concrete i_io_port implementation
 * based on the runtime type of an i_io_config.
 *
 * This is the ONLY file in ext_libs/io_helper/ (root level) that
 * includes protocol sub-folder headers (uart/uart_port.h, etc.).
 * All gnss_wrapper module code depends only on this factory plus
 * the common abstract contracts; it never includes sub-folder headers
 * directly.
 *
 * Design notes (SOLID):
 *  • Open/Closed – Adding a new channel type (e.g. SPI, TCP) requires
 *    only: a new spi/ sub-folder and one new case in io_port_factory.cpp.
 *    Zero changes to existing classes.
 *  • Dependency Inversion – gnss_controller depends on this factory
 *    and on i_io_port; it never instantiates uart_port or i2c_port
 *    directly, so it is insulated from implementation details.
 */

#ifndef IO_PORT_FACTORY_H
#define IO_PORT_FACTORY_H

#include "i_io_port.h"
#include "i_io_config.h"

#include <memory>

namespace io
{

class io_port_factory
{
public:
    /**
     * @brief Create and return the concrete i_io_port for the given config type.
     *
     * The returned port is freshly constructed and not yet open.
     * Call i_io_port::open(config) (or io_channel_wrapper::initialize(config))
     * on the result to begin I/O.
     *
     * @param config  Configuration whose channel_type() discriminator selects
     *                the concrete implementation.
     * @return A new shared_ptr<i_io_port> owning the implementation.
     * @throws std::invalid_argument for unsupported or unimplemented channel types.
     */
    static std::shared_ptr<i_io_port> create(const i_io_config& config);
};

} // namespace io

#endif // IO_PORT_FACTORY_H
