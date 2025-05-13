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

#include <fstream>

#include "solution.h"

namespace iopddl
{
    SolutionManager::SolutionManager(const Problem &problem) : SolutionManager(problem, createSolution(problem)) {}

    SolutionManager::SolutionManager(const Problem &problem, const Solution &solution)
        : problem(problem),
          solution(solution),
          usageTree(problem.max_time, problem.usage_limit)
    {
        for (NodeIdx nodeIdx = 0; nodeIdx < problem.nodes.size(); nodeIdx++)
        {
            const Node &node = problem.nodes[nodeIdx];

            // skip merged nodes
            if (node.dependency != -1)
                continue;

            const StrategyIdx strategyIdx = solution[nodeIdx];

            // add node cost
            cost += node.costs[strategyIdx];

            // initialise segment tree with usages
            for (int i = 0; i < node.intervals.size(); i++)
                usageTree.addToRange(node.intervals[i].first, node.intervals[i].second, node.usages[strategyIdx][i]);

            // add edge costs
            for (const auto &entry : node.edges)
            {
                const Edge &edge = *entry.second;
                if (edge.src < edge.dst)
                    cost += edge.costs[solution[edge.src]][solution[edge.dst]];
            }
        }
    }

    void SolutionManager::apply(const SolutionManager &other)
    {
        solution = other.solution;
        usageTree = other.usageTree;
        cost = other.cost;
    }

    TotalCost SolutionManager::getNodeCost(NodeIdx nodeIdx) const
    {
        const Node &node = problem.nodes[nodeIdx];
        TotalCost cost = node.costs[solution[nodeIdx]];
        for (auto edgePtr : node.edges)
        {
            const Edge &edge = *edgePtr.second;
            cost += edge.costs[solution[edge.src]][solution[edge.dst]];
        }
        return cost;
    }

    TotalCost SolutionManager::getEdgeCost(const Edge &edge) const
    {
        return edge.costs[solution[edge.src]][solution[edge.dst]];
    }

    bool SolutionManager::isFeasible()
    {
        return usageTree.getMaxValue() <= problem.usage_limit;
    }

    TimeIdx SolutionManager::getMaxValueIdx()
    {
        return usageTree.getMaxValueIdx();
    }

    void SolutionManager::setStrIdx(NodeIdx nodeIdx, StrategyIdx newStrIdx)
    {
        if (solution[nodeIdx] == newStrIdx)
            return;

        const Node &node = problem.nodes[nodeIdx];
        StrategyIdx oldStrIdx = solution[nodeIdx];

        // update memory usages
        for (int i = 0; i < node.intervals.size(); ++i)
        {
            auto interval = node.intervals[i];
            const TotalUsage memDiff = node.usages[newStrIdx][i] - node.usages[oldStrIdx][i];
            usageTree.addToRange(interval.first, interval.second, memDiff);
        }

        // update cost and strategy
        cost -= getNodeCost(nodeIdx);
        solution[nodeIdx] = newStrIdx;
        cost += getNodeCost(nodeIdx);
    }

    // creates a random solution, that is not necessarily feasible
    Solution SolutionManager::createSolution(const Problem &problem)
    {
        unsigned int seed = static_cast<unsigned int>(absl::ToUnixNanos(absl::Now()));
        std::srand(seed);
        Solution solution(problem.nodes.size());
        for (NodeIdx nodeIdx = 0; nodeIdx < problem.nodes.size(); nodeIdx++)
            solution[nodeIdx] = std::rand() % problem.nodes[nodeIdx].costs.size();
        return solution;
    }

    // propagates the solution to a dependent node
    void SolutionManager::propagateSolution(const NodeIdx nodeIdx)
    {
        auto &node = problem.nodes[nodeIdx];
        if (solution[node.dependency] == -1)
            // recurse if dependency is not yet resolved
            propagateSolution(node.dependency);
        // resolve dependency
        solution[nodeIdx] = node.mapping.value()[solution[node.dependency]];
    }

    // propagates the solution of all nodes to their dependencies
    void SolutionManager::propagateSolution()
    {
        for (NodeIdx nodeIdx = 0; nodeIdx < problem.nodes.size(); nodeIdx++)
            if (problem.nodes[nodeIdx].dependency != -1)
                // reset solution of dependent node
                solution[nodeIdx] = -1;
        for (NodeIdx nodeIdx = 0; nodeIdx < problem.nodes.size(); nodeIdx++)
            if (solution[nodeIdx] == -1)
                // propagate solution to dependent node
                propagateSolution(nodeIdx);
    }
}
