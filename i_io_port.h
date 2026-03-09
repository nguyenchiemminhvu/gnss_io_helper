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
 * i_io_port.h
 * ============================================================
 * Combined abstract interface for a full-duplex I/O channel.
 *
 * Inherits both i_io_reader and i_io_writer, and adds
 * lifecycle management (open / close / flush / drain) plus
 * configuration inspection.
 *
 * Design notes (SOLID):
 *  • Open/Closed  – New transports (e.g. USB-CDC, SPI, I2C, TCP)
 *    implement this interface without modifying existing code.
 *  • Liskov       – Any i_io_port implementation can substitute
 *    for any other without changing the consumer's behaviour.
 *  • Dependency Inversion – High-level components (e.g. gnss_controller)
 *    depend on this abstraction, not on uart_port (concrete class).
 */

#ifndef I_IO_PORT_H
#define I_IO_PORT_H

#include "i_io_reader.h"
#include "i_io_writer.h"
#include "i_io_config.h"

namespace io
{

class i_io_port : public i_io_reader, public i_io_writer
{
public:
    virtual ~i_io_port() = default;

    // -----------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------

    /**
     * @brief Open the channel described by `config` and apply protocol settings.
     *
     * Calling open() on an already-open port implicitly closes it first.
     *
     * @param[in] config  Channel configuration (concrete subtype of i_io_config).
     * @return true on success.
     * @throws io_open_error   if the device node / resource cannot be opened.
     * @throws io_config_error if protocol parameters cannot be applied.
     */
    virtual bool open(const i_io_config& config) = 0;

    /**
     * @brief Flush pending I/O, then close the channel.
     *
     * Safe to call on an already-closed channel (no-op).
     */
    virtual void close() = 0;

    /**
     * @brief Return whether the channel is currently open.
     */
    virtual bool is_open() const = 0;

    // -----------------------------------------------------------
    // Configuration inspection
    // -----------------------------------------------------------

    /**
     * @brief Return the configuration that was last successfully applied.
     *
     * Undefined behaviour if called before a successful open().
     */
    virtual const i_io_config& get_config() const = 0;

    // -----------------------------------------------------------
    // Buffer management
    // -----------------------------------------------------------

    /**
     * @brief Discard all data in the Tx and Rx buffers.
     *
     * For UART: equivalent to tcflush(fd, TCIOFLUSH).
     *
     * @return true on success, false if the channel is not open.
     */
    virtual bool flush() = 0;

    /**
     * @brief Block until all data in the Tx buffer has been sent.
     *
     * For UART: equivalent to tcdrain(fd).  Call after write_bytes() when
     * you need to guarantee the device has received the command before
     * reading its ACK.
     *
     * @return true on success, false if the channel is not open.
     */
    virtual bool drain() = 0;

    /**
     * @brief Close and re-open the channel with its current configuration.
     *
     * The close and re-open sequence is performed atomically under
     * the channel's internal mutex, so it is safe to call while a
     * concurrent read or write is in progress.
     *
     * @return true on success.
     * @throws io_open_error, io_config_error on failure.
     */
    virtual bool reconnect() = 0;

    /**
     * @brief Close the channel and re-open it with a new configuration.
     *
     * The close and re-open sequence is performed atomically under
     * the channel's internal mutex.
     *
     * @param new_config  Configuration to apply (concrete subtype of i_io_config).
     * @return true on success.
     * @throws io_open_error, io_config_error on failure.
     */
    virtual bool reconfigure(const i_io_config& new_config) = 0;

    // -----------------------------------------------------------
    // fd accessor – for poll()/epoll()/select() by external components
    // -----------------------------------------------------------
    /**
     * @brief Return the underlying POSIX file descriptor.
     *
     * Intended for use with poll(2)/epoll_ctl(2)/select(2) by external
     * components that need to monitor the channel for readability without
     * calling into the i_io_port read path directly.
     *
     * @note Valid only while is_open() == true.
     *       Returns -1 when the channel is closed; passing -1 to poll()
     *       yields POLLNVAL which is safe to detect and handle.
     * @note The caller MUST NOT close(), dup(), or dup2() this fd.
     *       Ownership remains with the concrete i_io_port implementation.
     * @note For channel types that do not expose a POSIX fd (e.g. pure
     *       software queues), implementations may always return -1.
     */
    virtual int get_fd() const noexcept = 0;
};

} // namespace io

#endif // I_IO_PORT_H
