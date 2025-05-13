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

#include "SegmentTree.h"

namespace iopddl
{
    // Propagate lazy updates
    void SegmentTree::propagate(TimeIdx node, TimeIdx start, TimeIdx end)
    {
        if (lazy[node] != 0)
        {
            // Apply the lazy update
            tree[node] += lazy[node];

            // Propagate to children if not a leaf node
            if (start != end)
            {
                lazy[node * 2] += lazy[node];
                lazy[node * 2 + 1] += lazy[node];
            }

            // Clear lazy value
            lazy[node] = 0;
        }
    }

    // Range update: Add `value` to range [l, r]
    void SegmentTree::range_update(TimeIdx node, TimeIdx start, TimeIdx end, TimeIdx l, TimeIdx r, TotalCost value)
    {
        propagate(node, start, end); // Ensure we apply any pending updates

        if (r < start || l > end)
            return; // Out of range

        if (l <= start && end <= r)
        { // Completely in range
            lazy[node] += value;
            propagate(node, start, end);
            return;
        }

        // Partial overlap: Recur on both children
        int mid = (start + end) / 2;
        range_update(node * 2, start, mid, l, r, value);
        range_update(node * 2 + 1, mid + 1, end, l, r, value);
        tree[node] = std::max(tree[node * 2], tree[node * 2 + 1]);
    }

    // Range query: Get min and max value in [l, r]
    TotalUsage SegmentTree::range_query(TimeIdx node, TimeIdx start, TimeIdx end, TimeIdx l, TimeIdx r)
    {
        propagate(node, start, end); // Ensure correct values

        if (r < start || l > end)
            return 0;

        if (l <= start && end <= r)
            return tree[node]; // Completely in range

        // Partial overlap: Query both children
        int mid = (start + end) / 2;
        TotalUsage leftMax = range_query(node * 2, start, mid, l, r);
        TotalUsage rightMax = range_query(node * 2 + 1, mid + 1, end, l, r);
        return std::max(leftMax, rightMax);
    }

    // Get the index of the max value
    TimeIdx SegmentTree::getMaxValueIdx(TimeIdx node, TimeIdx start, TimeIdx end)
    {
        if (start == end)
            return start;

        int mid = (start + end) / 2;
        propagate(node, start, end);
        propagate(node * 2, start, mid);
        propagate(node * 2 + 1, mid + 1, end);

        if (tree[node * 2] == tree[node])
            return getMaxValueIdx(node * 2, start, mid);
        else
            return getMaxValueIdx(node * 2 + 1, mid + 1, end);
    }

    // Constructor
    SegmentTree::SegmentTree(TimeIdx size, TotalCost u) : maxTime(size), upper_bound(u)
    {
        tree.resize(4 * maxTime, 0);
        lazy.resize(4 * maxTime, 0);
    }

    // Range update: Add `value` to range [l, r)
    void SegmentTree::addToRange(TimeIdx l, TimeIdx r, TotalCost value)
    {
        range_update(1, 0, maxTime - 1, l, r - 1, value);
    }

    // Get the maximum value in the whole range
    TotalCost SegmentTree::getMaxValue()
    {
        return range_query(1, 0, maxTime - 1, 0, maxTime - 1);
    }

    // Get the index of the maximum violation in the whole range
    TimeIdx SegmentTree::getMaxValueIdx()
    {
        return getMaxValueIdx(1, 0, maxTime - 1);
    }
};
