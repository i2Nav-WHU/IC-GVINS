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

#ifndef TIMECOST_H
#define TIMECOST_H

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>

class TimeCost {

public:
    TimeCost() {
        restart();
    }

    void restart() {
        start_     = absl::Now();
        is_finish_ = false;
    }

    void finish() {
        end_       = absl::Now();
        duration_  = end_ - start_;
        is_finish_ = true;
    }

    double costInSecond() {
        if (!is_finish_) {
            finish();
        }

        return absl::ToDoubleSeconds(duration_);
    }

    std::string costInSecond(const std::string &header) {
        auto cost = costInSecond();
        return absl::StrFormat("%s %0.6lf seconds", header.c_str(), cost);
    }

    double costInMillisecond() {
        if (!is_finish_) {
            finish();
        }
        return absl::ToDoubleMilliseconds(duration_);
    }

    std::string costInMillisecond(const std::string &header) {
        auto cost = costInMillisecond();
        return absl::StrFormat("%s %0.3lf milliseconds", header.c_str(), cost);
    }

private:
    absl::Time     start_, end_;
    absl::Duration duration_;

    bool is_finish_{false};
};

#endif // TIMECOST_H
