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

#ifndef SOLUTION_H
#define SOLUTION_H

#include "iopddl.h"
#include "SegmentTree.h"
#include <string>

namespace iopddl
{
    class SolutionManager
    {
    public:
        const Problem &problem;
        Solution solution;
        SegmentTree usageTree;
        TotalCost cost = 0;

        SolutionManager(const Problem &problem);
        SolutionManager(const Problem &problem, const Solution &solution);
        ~SolutionManager() = default;

        void apply(const SolutionManager &other);
        TotalCost getNodeCost(NodeIdx nodeIdx) const;
        TotalCost getEdgeCost(const Edge &edge) const;
        bool isFeasible();
        TimeIdx getMaxValueIdx();
        void setStrIdx(NodeIdx nodeIdx, StrategyIdx newStrIdx);
        void propagateSolution();

    private:
        void propagateSolution(NodeIdx nodeIdx);
        Solution createSolution(const Problem &problem);
    };
}

#endif // SOLUTION_H
