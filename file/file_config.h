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
 * file_config.h
 * ============================================================
 * Record-file replay channel configuration deriving from i_io_config.
 *
 * Used to replay previously recorded binary GNSS data for offline
 * testing and simulation.
 */

#ifndef FILE_CONFIG_H
#define FILE_CONFIG_H

#include "../i_io_config.h"

#include <string>

namespace io
{

/**
 * @brief Record-file replay channel configuration.
 *
 * When used in replay mode (writable == false) the port opens the file
 * read-only and feeds data to the UBX/NMEA parser at full speed.
 * When writable == true the file is created/truncated and opened for
 * both reading and writing (useful for GNSS data recording).
 *
 * loop == true causes read operations to wrap around to the start of
 * the file when EOF is reached, enabling continuous replay.
 *
 * throttle_us controls an optional inter-read sleep (microseconds) to
 * simulate real-time UART throughput during replay.  0 = no throttle.
 */
struct file_config : public i_io_config
{
    std::string file_path;              ///< Path to the binary recording file
    bool        loop         = false;   ///< Wrap to file start on EOF (replay mode)
    bool        writable     = false;   ///< Open with write permission (recording mode)
    int         throttle_us  = 0;       ///< Optional inter-read sleep in microseconds

    io_channel_type channel_type() const noexcept override
    {
        return io_channel_type::file;
    }

    // -------------------------------------------------------
    // Factory helpers
    // -------------------------------------------------------

    /**
     * @brief Open a pre-recorded binary file for replay (read-only, looping).
     *
     * @param path        Path to the recorded binary file.
     * @param loop        true = replay indefinitely from the beginning.
     * @param throttle_us Microseconds to sleep between reads (0 = full speed).
     */
    static file_config replay(const std::string& path,
                              bool               loop        = false,
                              int                throttle_us = 0)
    {
        file_config cfg;
        cfg.file_path    = path;
        cfg.loop         = loop;
        cfg.writable     = false;
        cfg.throttle_us  = throttle_us;
        return cfg;
    }

    /**
     * @brief Create / overwrite a file for GNSS data recording (write + read).
     *
     * @param path  Path to the output file (created if absent, truncated if present).
     */
    static file_config record(const std::string& path)
    {
        file_config cfg;
        cfg.file_path = path;
        cfg.writable  = true;
        cfg.loop      = false;
        return cfg;
    }

    /**
     * @brief Fully custom file channel configuration.
     *
     * Convenience factory when none of the named presets fit, e.g.
     * a writable looping file or a throttled one-shot replay.
     *
     * @param path        Path to the binary file.
     * @param writable    true = open with write permission (recording mode).
     * @param loop        true = wrap to file start on EOF (replay mode).
     * @param throttle_us Microseconds to sleep between reads (0 = full speed).
     */
    static file_config custom(const std::string& path,
                              bool               writable    = false,
                              bool               loop        = false,
                              int                throttle_us = 0)
    {
        file_config cfg;
        cfg.file_path   = path;
        cfg.writable    = writable;
        cfg.loop        = loop;
        cfg.throttle_us = throttle_us;
        return cfg;
    }
};

} // namespace io

#endif // FILE_CONFIG_H
