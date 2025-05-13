/*****************************************************************************
 * Copyright 2025 Stephan Spengler and Samuel Grahn                          *
 *                                                                           *
 * This file is part of the 2025 IOPDDL contest submission                   *
 * by Stephan Spengler and Samuel Grahn.                                     *
 *                                                                           *
 * This program is free software:                                            *
 * you can redistribute it and/or modify it under the terms of the GNU       *
 * General Public License as published by the Free Software Foundation,      *
 * either version 3 of the License, or (at your option) any later version.   *
 *                                                                           *
 * This program is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                      *
 * See the GNU General Public License for more details.                      *
 *                                                                           *
 * You should have received a copy of the GNU General Public License along   *
 * with this program. If not, see <https://www.gnu.org/licenses/>.           *
 ****************************************************************************/

#ifndef PHASE_H
#define PHASE_H
#include <ostream>
#include <stdint.h>

enum Action : uint32_t
{
    RANDOM_WALK = 1 << 0,
    ELIMINATE_EDGE = 1 << 1,
    ONE_MAX = 1 << 2,
    TWO_MAX = 1 << 3,
};

constexpr enum Action operator|(const enum Action self, const enum Action other)
{
    return (enum Action)((uint32_t)(self) | (uint32_t)other);
}

struct PhaseConfig
{
    Action actions;

    // Real timeout is global_timeout * timeout_factor
    double timeout_factor;

    // For search, how many solutions per thread
    // For other phases, how many solutions to use this phase in total
    int solution_count;

    PhaseConfig(Action a, double to, int sc) : actions(a), timeout_factor(to), solution_count(sc) {}
};

inline std::ostream &operator<<(std::ostream &os, const Action &a)
{
    os << "[";
    bool has_thing = false;
    if (a & RANDOM_WALK)
    {
        if (has_thing)
            os << "|";
        os << "RandomWalk";
        has_thing = true;
    }
    if (a & ELIMINATE_EDGE)
    {
        if (has_thing)
            os << "|";
        os << "EliminateEdge";
        has_thing = true;
    }
    if (a & ONE_MAX)
    {
        if (has_thing)
            os << "|";
        os << "1-max";
        has_thing = true;
    }
    if (a & TWO_MAX)
    {
        if (has_thing)
            os << "|";
        os << "2-max";
        has_thing = true;
    }
    return os << "]";
}

inline std::ostream &operator<<(std::ostream &os, const PhaseConfig &pc)
{
    return os << pc.actions << ", max " << pc.solution_count << " solutions";
}

// Timeouts should sum to <= 1 or it will exceed the global timeout
const PhaseConfig phases[] = {
    PhaseConfig(RANDOM_WALK | ELIMINATE_EDGE | ONE_MAX, 0.1, 128),
    PhaseConfig(ELIMINATE_EDGE | ONE_MAX, 0.1, 16),
    PhaseConfig(ELIMINATE_EDGE | ONE_MAX, 0.15, 1),
    PhaseConfig(ELIMINATE_EDGE | ONE_MAX | TWO_MAX, 0.15, 1),
    // We save 0.5% of the execution time for cleanup
    PhaseConfig(ELIMINATE_EDGE | TWO_MAX, 0.495, 1),
};

const int num_phases = sizeof(phases) / sizeof(PhaseConfig);

#endif // PHASE_H
