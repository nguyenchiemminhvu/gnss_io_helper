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
 * i_io_config.h
 * ============================================================
 * Polymorphic base for all I/O channel configurations.
 *
 * Protocol-specific configuration structs (uart_config, i2c_config,
 * file_config, …) derive from this base so that higher-level components
 * (gnss_receiver, gnss_controller) can accept any channel type without
 * being coupled to a specific transport.
 *
 * Design notes (SOLID):
 *  • Open/Closed  – Adding a new channel type (e.g. SPI, TCP) requires
 *    only a new concrete config struct; no existing code changes.
 *  • Dependency Inversion – gnss_receiver depends on i_io_config, not on
 *    uart_config directly.
 */

#ifndef I_IO_CONFIG_H
#define I_IO_CONFIG_H

namespace io
{

/**
 * @brief Discriminator for all concrete i_io_config implementations.
 *
 * Used by io_port_factory to select the correct i_io_port implementation,
 * and by uart_port::open() to validate that the supplied config is UART.
 */
enum class io_channel_type
{
    uart,
    i2c,
    file,
    udp,
    tcp,
};

/**
 * @brief Polymorphic base for all I/O channel configurations.
 *
 * Concrete protocol configs (uart_config, i2c_config, file_config)
 * derive from this struct and implement channel_type().
 */
struct i_io_config
{
    virtual ~i_io_config() = default;

    /**
     * @brief Return the channel type discriminator for this configuration.
     *
     * Implementations must return the corresponding io_channel_type value.
     */
    virtual io_channel_type channel_type() const noexcept = 0;
};

} // namespace io

#endif // I_IO_CONFIG_H
