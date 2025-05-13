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

#ifndef SEGMENT_TREE_H
#define SEGMENT_TREE_H

#include <vector>
#include <limits.h>

#include "iopddl.h"

namespace iopddl
{
    class SegmentTree
    {
    private:
        TimeIdx maxTime;
        std::vector<TotalUsage> tree;
        std::vector<TotalCost> lazy;
        int upper_bound;

        void propagate(TimeIdx node, TimeIdx start, TimeIdx end);
        void range_update(TimeIdx node, TimeIdx start, TimeIdx end, TimeIdx l, TimeIdx r, TotalCost value);
        TotalUsage range_query(TimeIdx node, TimeIdx start, TimeIdx end, TimeIdx l, TimeIdx r);
        TimeIdx getMaxValueIdx(TimeIdx node, TimeIdx start, TimeIdx end);

    public:
        SegmentTree(TimeIdx size, TotalCost u);
        void addToRange(TimeIdx l, TimeIdx r, TotalCost value);
        TotalCost getMaxValue();
        TimeIdx getMaxValueIdx();
    };
}

#endif // SEGMENT_TREE_H
