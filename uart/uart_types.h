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
 * uart_types.h
 * ============================================================
 * Foundational enumerations that describe every aspect of a
 * UART frame configuration.  These types are used by
 * uart_config and throughout the UART implementation.
 *
 * Targeting: Linux (Yocto / embedded)
 */

#ifndef UART_TYPES_H
#define UART_TYPES_H

#include <cstdint>

namespace io
{

/**
 * @brief Standard UART baud rates.
 *
 * The underlying value equals the numeric baud rate in bits-per-second.
 * uart_port::baud_to_speed() maps these to the POSIX termios Bxxx
 * constants before passing them to cfsetispeed / cfsetospeed.
 *
 * Rates >= B_460800 are Linux-specific termios extensions and are NOT
 * available on all UART hardware drivers.  Verify with your BSP before use.
 */
enum class baud_rate : uint32_t
{
    B_50       =       50,
    B_75       =       75,
    B_110      =      110,
    B_134      =      134,
    B_150      =      150,
    B_200      =      200,
    B_300      =      300,
    B_600      =      600,
    B_1200     =     1200,
    B_1800     =     1800,
    B_2400     =     2400,
    B_4800     =     4800,
    B_9600     =     9600,   ///< u-blox GNSS default
    B_19200    =    19200,
    B_38400    =    38400,
    B_57600    =    57600,
    B_115200   =   115200,   ///< u-blox GNSS high-speed
    B_230400   =   230400,
    B_460800   =   460800,
    B_921600   =   921600,
    B_1000000  =  1000000,
    B_1152000  =  1152000,
    B_1500000  =  1500000,
    B_2000000  =  2000000,
    B_2500000  =  2500000,
    B_3000000  =  3000000,
    B_3500000  =  3500000,
    B_4000000  =  4000000,
};

/**
 * @brief Number of data bits per UART character frame.
 */
enum class data_bits : uint8_t
{
    FIVE  = 5,
    SIX   = 6,
    SEVEN = 7,
    EIGHT = 8,   ///< Standard for UBX binary protocol
};

/**
 * @brief Parity check mode.
 */
enum class parity : uint8_t
{
    NONE = 0,   ///< No parity (most common for GNSS)
    ODD  = 1,
    EVEN = 2,
};

/**
 * @brief Number of stop bits appended to each character frame.
 */
enum class stop_bits : uint8_t
{
    ONE = 1,   ///< Standard for UBX binary protocol
    TWO = 2,
};

/**
 * @brief Flow control strategy.
 */
enum class flow_control : uint8_t
{
    NONE     = 0,   ///< No flow control (typical for GNSS chips)
    HARDWARE = 1,   ///< RTS / CTS hardware handshake
    SOFTWARE = 2,   ///< XON / XOFF in-band signalling
};

} // namespace io

#endif // UART_TYPES_H
