/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef FILEBASE_H
#define FILEBASE_H

#include <fstream>
#include <vector>

using std::string;
using std::vector;

class FileBase {

public:
    enum FileType {
        TEXT   = 0,
        BINARY = 1,
    };

    FileBase() = default;
    ~FileBase() {
        if (isOpen())
            filefp_.close();
    }

    void close() {
        filefp_.close();
    }

    bool isOpen() {
        return filefp_.is_open();
    }

    bool isEof() {
        return filefp_.eof();
    }

    std::fstream &fstream() {
        return filefp_;
    }

    int columns() const {
        return columns_;
    }

    void flush() {
        filefp_.flush();
    }

protected:
    std::fstream filefp_;
    int filetype_ = TEXT;

    int columns_ = 0;
};

#endif // FILEBASE_H
