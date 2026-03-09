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
 * i2c_config.h
 * ============================================================
 * I2C (DDC) channel configuration deriving from i_io_config.
 *
 * Used with u-blox DDC (I2C) interface on supported chips
 * (default address 0x42).
 */

#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include "../i_io_config.h"

#include <cstdint>
#include <string>

namespace io
{

// -------------------------------------------------------
// u-blox DDC (I2C) register map
// -------------------------------------------------------

/// MSB of the "bytes available" counter (u-blox DDC register 0xFD).
constexpr uint8_t I2C_DDC_REG_BYTES_AVAIL_MSB = 0xFD;

/// LSB of the "bytes available" counter (u-blox DDC register 0xFE).
constexpr uint8_t I2C_DDC_REG_BYTES_AVAIL_LSB = 0xFE;

/// Data-stream read register (u-blox DDC register 0xFF).
/// Writing this register address then reading N bytes delivers queued UBX/NMEA data.
constexpr uint8_t I2C_DDC_REG_DATA_STREAM     = 0xFF;

/// Data-stream write register (u-blox DDC register 0x00).
/// Prepend this byte to any UBX command payload before writing.
constexpr uint8_t I2C_DDC_REG_DATA_WRITE      = 0x00;

/**
 * @brief I2C (DDC) channel configuration.
 *
 * Describes the Linux i2c-dev adapter to open and the u-blox slave
 * address to target.  read_timeout_ms controls how long available()
 * waits for data before returning 0 in the worker-loop polling pattern.
 */
struct i2c_config : public i_io_config
{
    std::string device_path;             ///< e.g. "/dev/i2c-1", "/dev/i2c-3"
    uint8_t     address         = 0x42;  ///< u-blox DDC default slave address
    int         read_timeout_ms = 100;   ///< poll timeout used by available(); 0 = no wait

    io_channel_type channel_type() const noexcept override
    {
        return io_channel_type::i2c;
    }

    // -------------------------------------------------------
    // Factory helpers
    // -------------------------------------------------------

    /**
     * @brief u-blox DDC default: /dev/i2c-1, address 0x42, 100 ms poll timeout.
     *
     * @param dev   Linux i2c-dev adapter path, e.g. "/dev/i2c-1".
     * @param addr  I2C slave address (u-blox default 0x42).
     */
    static i2c_config ddc_default(const std::string& dev, uint8_t addr = 0x42)
    {
        i2c_config cfg;
        cfg.device_path     = dev;
        cfg.address         = addr;
        cfg.read_timeout_ms = 100;
        return cfg;
    }

    /**
     * @brief Custom I2C configuration with explicit address and poll timeout.
     *
     * Convenience factory when the target device uses a non-default slave
     * address or requires a specific poll timeout.
     *
     * @param dev             Linux i2c-dev adapter path, e.g. "/dev/i2c-3".
     * @param addr            I2C slave address (u-blox default 0x42).
     * @param read_timeout_ms Poll timeout in ms used by available(); 0 = no wait.
     */
    static i2c_config custom(const std::string& dev, uint8_t addr = 0x42, int read_timeout_ms = 100)
    {
        i2c_config cfg = ddc_default(dev, addr);
        cfg.read_timeout_ms = read_timeout_ms;
        return cfg;
    }
};

} // namespace io

#endif // I2C_CONFIG_H
