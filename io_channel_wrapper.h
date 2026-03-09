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
 * io_channel_wrapper.h
 * ============================================================
 * High-level wrapper that manages an I/O channel session for use
 * by a gnss_controller (or any other hardware-adapter component).
 *
 * Responsibilities:
 *  • Own the i_io_port implementation (dependency-injected).
 *  • Expose initialize / shutdown / reconfigure for lifecycle
 *    management with clear semantics.
 *  • Forward all read / write operations to the underlying port
 *    so callers never touch the raw interface directly.
 *  • Provide reconnect() to recover from transient I/O errors
 *    without requiring the caller to manage the fd lifecycle.
 *
 * Thread-safety:
 *  All operations are thread-safe.  The mutex lives inside the
 *  concrete i_io_port implementation (the layer that owns the fd),
 *  so protection is applied exactly where the shared resource is
 *  accessed.  io_channel_wrapper itself is a stateless delegation
 *  facade — it holds no mutable state of its own and therefore
 *  needs no additional locking.
 *
 * Design notes (SOLID):
 *  • Single Responsibility – manages context & lifecycle only;
 *    raw I/O logic and synchronisation live in the concrete port.
 *  • Dependency Inversion  – depends on i_io_port abstraction,
 *    not on uart_port directly; callers inject the implementation
 *    (facilitates unit testing with mock ports).
 *  • Open/Closed – new i_io_port implementations slot in
 *    without touching io_channel_wrapper.
 */

#ifndef IO_CHANNEL_WRAPPER_H
#define IO_CHANNEL_WRAPPER_H

#include "i_io_port.h"
#include "i_io_config.h"

#include <memory>
#include <string>

namespace io
{

class io_channel_wrapper
{
public:
    /**
     * @brief Construct the wrapper with an injected port implementation.
     *
     * The wrapper holds a weak_ptr so the port lifetime is controlled by
     * the caller (typically shared between the receiver and the transport).
     *
     * @param port  Concrete i_io_port to manage (shared with other consumers).
     */
    explicit io_channel_wrapper(std::shared_ptr<i_io_port> port);

    /**
     * @brief Destructor – calls shutdown() if the port is still open.
     */
    ~io_channel_wrapper();

    // Non-copyable (holds a weak_ptr and delegates to a mutex-guarded port)
    io_channel_wrapper(const io_channel_wrapper&)            = delete;
    io_channel_wrapper& operator=(const io_channel_wrapper&) = delete;

    // -----------------------------------------------------------
    // Lifecycle management
    // -----------------------------------------------------------

    /**
     * @brief Open the channel with the given configuration.
     *
     * @param config  Channel configuration (concrete subtype of i_io_config).
     * @return true on success.
     * @throws io_open_error, io_config_error on failure.
     */
    bool initialize(const i_io_config& config);

    /**
     * @brief Flush and close the channel.
     *
     * Safe to call multiple times (no-op when already closed).
     */
    void shutdown();

    /**
     * @brief Close and re-open the channel with its current configuration.
     *
     * Delegates directly to i_io_port::reconnect(), which performs
     * the close/open sequence atomically under the port's internal mutex.
     *
     * @return true on success.
     * @throws io_open_error, io_config_error on failure.
     */
    bool reconnect();

    /**
     * @brief Apply a new configuration to the already-open channel.
     *
     * Delegates directly to i_io_port::reconfigure(), which performs
     * the close/open sequence atomically under the port's internal mutex.
     *
     * @param new_config  Configuration to apply.
     * @return true on success.
     * @throws io_open_error, io_config_error on failure.
     */
    bool reconfigure(const i_io_config& new_config);

    // -----------------------------------------------------------
    // Status
    // -----------------------------------------------------------

    /**
     * @brief Return true if the channel is open and ready for I/O.
     */
    bool is_ready() const;

    /**
     * @brief Return the configuration currently applied to the channel.
     *
     * Caller must ensure is_ready() before calling this.
     */
    const i_io_config& get_config() const;

    // -----------------------------------------------------------
    // fd accessor – for poll()/epoll()/select() by external components
    // -----------------------------------------------------------
    /**
     * @brief Return the underlying POSIX file descriptor of the managed port.
     *
     * Allows external components (e.g. GNSS HAL frame pump) to use
     * poll(2)/epoll_ctl(2)/select(2) directly on the channel fd without
     * going through the io_channel_wrapper read path.
     *
     * @note Valid only while is_ready() == true.
     *       Returns -1 when the port is closed; poll() will then
     *       return POLLNVAL which the caller can detect safely.
     */
    int get_fd() const noexcept;

    // -----------------------------------------------------------
    // Read operations (thread-safe)
    // -----------------------------------------------------------
    ssize_t read_byte(char& c);
    ssize_t read_chars(char* buf, std::size_t len);
    ssize_t read_uchars(unsigned char* buf, std::size_t len);
    ssize_t read_string(std::string& str, std::size_t max_len);
    ssize_t read_bytes(std::vector<uint8_t>& buf, std::size_t max_len);
    int     available() const;

    // -----------------------------------------------------------
    // Write operations (thread-safe)
    // -----------------------------------------------------------
    ssize_t write_byte(char c);
    ssize_t write_chars(const char* buf, std::size_t len);
    ssize_t write_uchars(const unsigned char* buf, std::size_t len);
    ssize_t write_string(const std::string& str);
    ssize_t write_bytes(const std::vector<uint8_t>& buf);

    // -----------------------------------------------------------
    // Buffer management (thread-safe)
    // -----------------------------------------------------------
    bool flush();
    bool drain();

private:
    std::weak_ptr<i_io_port>   port_;
};

} // namespace io

#endif // IO_CHANNEL_WRAPPER_H
