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

#include "search.h"
#include "absl/numeric/int128.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "phase.h"
#include "solution.h"
#include <bit>
#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace iopddl
{
  void eliminateExpensiveEdge(const Problem &problem, SolutionManager &solMgr, absl::Duration timeout)
  {
    absl::Time start = absl::Now();
    unsigned int seed = static_cast<unsigned int>(absl::ToUnixNanos(absl::Now()));
    std::srand(seed);

    Solution &solution = solMgr.solution;

    TotalCost totalCost = 0;
    for (NodeIdx src = 0; src < problem.nodes.size(); src++)
      if (problem.nodes[src].dependency == -1)
        for (auto &[dst, edge] : problem.nodes[src].edges)
          totalCost += problem.nodes[src].costs[solution[src]] + (*edge).costs[solution[src]][solution[dst]] + problem.nodes[dst].costs[solution[dst]];

    // choose a random edge, favouring expensive edges
    TotalCost rnd = totalCost * std::rand() / RAND_MAX;
    for (NodeIdx src = 0; src < problem.nodes.size(); src++)
      if (problem.nodes[src].dependency == -1)
        for (auto &[dst, edge] : problem.nodes[src].edges)
        {
          if (absl::Now() - start > timeout)
          {
            return;
          }

          totalCost -= problem.nodes[src].costs[solution[src]] + (*edge).costs[solution[src]][solution[dst]] + problem.nodes[dst].costs[solution[dst]];
          if (rnd >= totalCost)
          {
            double totalInvCost = 0;
            for (StrategyIdx strategyIdx1 = 0; strategyIdx1 < problem.nodes[src].costs.size(); strategyIdx1++)
              for (StrategyIdx strategyIdx2 = 0; strategyIdx2 < problem.nodes[dst].costs.size(); strategyIdx2++)
                totalInvCost += 1.0 / static_cast<double>(1 + problem.nodes[src].costs[strategyIdx1] + (*edge).costs[strategyIdx1][strategyIdx2] + problem.nodes[dst].costs[strategyIdx2]);

            // choose a random pair of strategies, favouring cheap strategies
            double rnd = totalInvCost * std::rand() / RAND_MAX;
            for (StrategyIdx strategyIdx1 = 0; strategyIdx1 < problem.nodes[src].costs.size(); strategyIdx1++)
              for (StrategyIdx strategyIdx2 = 0; strategyIdx2 < problem.nodes[dst].costs.size(); strategyIdx2++)
              {
                totalInvCost -= 1.0 / static_cast<double>(1 + problem.nodes[src].costs[strategyIdx1] + (*edge).costs[strategyIdx1][strategyIdx2] + problem.nodes[dst].costs[strategyIdx2]);
                if (rnd >= totalInvCost)
                {
                  solMgr.setStrIdx(src, strategyIdx1);
                  solMgr.setStrIdx(dst, strategyIdx2);
                  return;
                }
              }
            return;
          }
        }
  }

  void randomWalk(const Problem &problem, SolutionManager &solMgr, absl::Duration timeout)
  {
    unsigned int seed = static_cast<unsigned int>(absl::ToUnixNanos(absl::Now()));
    std::srand(seed);
    absl::Time start = absl::Now();

    // find most expensive node (including incident edges)
    std::vector<std::pair<NodeIdx, TotalCost>> nodeCosts;
    TotalCost totalCost = 0;
    for (NodeIdx nodeIdx = 0; nodeIdx < problem.nodes.size(); ++nodeIdx)
      if (problem.nodes[nodeIdx].dependency == -1)
      {
        TotalCost cost = solMgr.getNodeCost(nodeIdx);
        totalCost += cost;
        nodeCosts.emplace_back(nodeIdx, cost);
      }
    std::sort(nodeCosts.begin(), nodeCosts.end(), [](const auto &a, const auto &b)
              { return a.second > b.second; });

    // choose a random node, favouring expensive nodes
    TotalCost rnd = totalCost * std::rand() / RAND_MAX;
    for (const auto &nodeCost : nodeCosts)
    {
      NodeIdx nodeIdx = nodeCost.first;
      TotalCost cost = nodeCost.second;
      totalCost -= cost;
      if (rnd >= totalCost)
      {
        auto handled = std::unordered_set<NodeIdx>();
        auto frontier = std::vector<NodeIdx>();
        frontier.push_back(nodeIdx);

        for (int i = 0; i < 20 && !frontier.empty(); i++)
        {
          if (absl::Now() - start > timeout)
            return;

          // choose a random node from the frontier
          int idx = std::rand() % frontier.size();
          NodeIdx current = frontier[idx];
          frontier[idx] = frontier.back();
          frontier.pop_back();
          handled.insert(current);

          // choose a random strategy for the current node
          const Node &node = problem.nodes[current];
          StrategyIdx strategyIdx = std::rand() % node.costs.size();
          solMgr.setStrIdx(current, strategyIdx);

          for (const auto &[dst, edge] : node.edges)
            if (handled.find(dst) == handled.end())
              frontier.push_back(dst);
        }
        return;
      }
    }
  }

  // returns the memory usage of node at timeIdx for each strategy
  // uses caching to avoid recomputing
  std::vector<TotalUsage> getCachedUsages(TimeIdx timeIdx, Node &node)
  {
    auto cache = node.cache.find(timeIdx);
    if (cache != node.cache.end())
      return cache->second;

    std::vector<TotalUsage> usages;
    for (StrategyIdx strIdx = 0; strIdx < node.usages.size(); strIdx++)
    {
      TotalUsage usage = 0;
      for (int i = 0; i < node.intervals.size(); ++i)
      {
        Interval interval = node.intervals[i];
        if (interval.first <= timeIdx && interval.second > timeIdx)
          usage += node.usages[strIdx][i];
      }
      usages.push_back(usage);
    }
    node.cache.emplace(timeIdx, usages);
    return usages;
  }

  // returns the nodes that are active at timeIdx
  // uses caching to avoid recomputing
  std::vector<NodeIdx> getCachedNodes(Problem &problem, TimeIdx timeIdx)
  {
    auto cache = problem.cache.find(timeIdx);
    if (cache != problem.cache.end())
      return cache->second;

    std::vector<NodeIdx> nodes;
    for (const Node &node : problem.nodes)
      if (node.dependency == -1)
        for (const Interval &interval : node.intervals)
          if (interval.first <= timeIdx && interval.second > timeIdx)
          {
            nodes.emplace_back(node.idx);
            continue;
          }
    problem.cache.emplace(timeIdx, nodes);
    return nodes;
  }

  // tries to make the solution feasible
  void makeFeasible(Problem &problem, SolutionManager &solMgr, absl::Duration timeout)
  {
    unsigned int seed = static_cast<unsigned int>(absl::ToUnixNanos(absl::Now()));
    std::srand(seed);

    auto start_time = absl::Now();
    while (!solMgr.isFeasible())
    {
      if (absl::Now() - start_time > timeout)
        return;

      TimeIdx maxValueIdx = solMgr.getMaxValueIdx();
      const std::vector<NodeIdx> &violations = getCachedNodes(problem, maxValueIdx);

      // choose a random node that is active at the maximum violation
      Node &node = problem.nodes[violations[std::rand() % violations.size()]];
      auto cached = getCachedUsages(maxValueIdx, node);
      TotalUsage currentUsage = cached[solMgr.solution[node.idx]];

      std::vector<std::pair<StrategyIdx, TotalUsage>> usages;
      double totalInvUsage = 0.0;
      for (StrategyIdx strIdx = 0; strIdx < node.costs.size(); ++strIdx)
      {
        TotalUsage usage = cached[strIdx];
        if (usage < currentUsage)
        {
          usages.emplace_back(strIdx, usage);
          totalInvUsage += 1.0 / static_cast<double>(usage + 1);
        }
      }

      if (usages.empty())
        continue;

      // choose a random strategy, favouring cheap (wrt memory) strategies
      double rnd = totalInvUsage * std::rand() / RAND_MAX;
      for (auto &entry : usages)
      {
        rnd -= 1.0 / static_cast<double>(entry.second + 1);
        if (rnd < 0)
        {
          solMgr.setStrIdx(node.idx, entry.first);
          break;
        }
      }
    }
  }

  // assumes feasible solution
  void makeCheap(const Problem &problem, SolutionManager &solMgr, absl::Duration timeout)
  {
    unsigned int seed = static_cast<unsigned int>(absl::ToUnixNanos(absl::Now()));
    std::srand(seed);

    // (arbitrarily) cap the maximum cost to 1/4 of the current cost, previously 1e18
    TotalCost curr_max = solMgr.cost / 4;

    auto start_time = absl::Now();
    while (true)
    {
      if (absl::Now() - start_time > timeout)
        return;

      // sort nodes by cost
      std::vector<std::pair<NodeIdx, TotalCost>> nodeList;
      TotalCost totalCost = 0;
      for (const Node &node : problem.nodes)
        if (node.dependency == -1)
        {
          TotalCost cost = solMgr.getNodeCost(node.idx);
          totalCost += cost;
          nodeList.emplace_back(node.idx, cost);
        }
      std::sort(nodeList.begin(), nodeList.end(), [](const auto &a, const auto &b)
                { return a.second > b.second; });

      SolutionManager cpy = solMgr;
      bool improved = false;

      // consider nodes from most expensive to cheapest
      for (auto &entry : nodeList)
      {
        if (absl::Now() - start_time > timeout)
          return;

        const Node &node = problem.nodes[entry.first];

        // set the strategy to the cheapest feasible one
        for (StrategyIdx strategyIdx = 0; strategyIdx < node.costs.size(); strategyIdx++)
          if (node.costs[strategyIdx] < curr_max)
          {
            cpy.setStrIdx(node.idx, strategyIdx);
            if (cpy.isFeasible() && cpy.cost < solMgr.cost)
            {
              solMgr.apply(cpy);
              improved = true;
            }
          }
        cpy.setStrIdx(node.idx, solMgr.solution[node.idx]);
      }

      if (!improved)
        return;
    }
  }

  // assumes feasible solution
  void makeCheaper(const Problem &problem, SolutionManager &solMgr, absl::Duration timeout)
  {
    unsigned int seed = static_cast<unsigned int>(absl::ToUnixNanos(absl::Now()));
    std::srand(seed);

    // (arbitrarily) cap the maximum cost to 1/4 of the current cost, previously 1e18
    TotalCost curr_max = solMgr.cost / 4;

    auto start_time = absl::Now();
    while (true)
    {
      if (absl::Now() - start_time > timeout)
        return;

      // sort edges by cost
      std::vector<std::pair<std::shared_ptr<Edge>, TotalCost>> edgeList;
      TotalCost totalCost = 0;
      for (const Node &node : problem.nodes)
        if (node.dependency == -1)
          for (auto &[_, edgePtr] : node.edges)
          {
            TotalCost cost = solMgr.getEdgeCost(*edgePtr);
            totalCost += cost;
            edgeList.emplace_back(edgePtr, cost);
          }
      std::sort(edgeList.begin(), edgeList.end(), [](const auto &a, const auto &b)
                { return a.second > b.second; });

      SolutionManager cpy = solMgr;
      bool improved = false;

      // consider edges from most expensive to cheapest
      for (auto &entry : edgeList)
      {
        if (absl::Now() - start_time > timeout)
          return;

        const Edge &edge = *entry.first;
        const Node &node1 = problem.nodes[edge.src];
        const Node &node2 = problem.nodes[edge.dst];

        // set the pair of strategies to the cheapest feasible one
        for (StrategyIdx strategyIdx1 = 0; strategyIdx1 < node1.costs.size(); strategyIdx1++)
          if (node1.costs[strategyIdx1] < curr_max)
            for (StrategyIdx strategyIdx2 = 0; strategyIdx2 < node2.costs.size(); strategyIdx2++)
              if (node2.costs[strategyIdx2] < curr_max)
                if (edge.costs[strategyIdx1][strategyIdx2] < curr_max)
                {
                  cpy.setStrIdx(node1.idx, strategyIdx1);
                  cpy.setStrIdx(node2.idx, strategyIdx2);
                  if (cpy.isFeasible() && cpy.cost < solMgr.cost)
                  {
                    solMgr.apply(cpy);
                    improved = true;
                  }
                }
        cpy.setStrIdx(node1.idx, solMgr.solution[node1.idx]);
        cpy.setStrIdx(node2.idx, solMgr.solution[node2.idx]);
      }

      if (!improved)
        return;
    }
  }

  std::shared_ptr<SolutionManager> findSolution(Problem &problem, absl::Duration timeout)
  {
    // generates random solution
    auto solMgr = std::make_shared<SolutionManager>(problem);

    absl::Time start = absl::Now();
    makeFeasible(problem, *solMgr, timeout);
    if (!solMgr->isFeasible())
      return nullptr;

    absl::Time cont = absl::Now();
    absl::Duration remaining = timeout - (cont - start);
    // This makes the selection of "best candidates" more effective.
    makeCheaper(problem, *solMgr, remaining);

    if (!solMgr->isFeasible())
      return nullptr;
    return solMgr;
  }

  std::shared_ptr<SolutionManager> improveSolution(Problem &problem, const Solution &solution, Action actions, int fails, absl::Duration timeout)
  {
    auto solMgr = std::make_shared<SolutionManager>(problem, solution);

    // Number of phases, this compiles to a hardware instruction on x86.
    int num_actions = std::popcount((uint32_t)actions);

    // If we fail a lot, we are likely stuck in a minimum.
    // We should *heat up more* by increasing the number of edges we eliminate.
    const EdgeIdx eliminationCount = (EdgeIdx)((problem.edge_count / 300.) * ((fails / 200.) + 10));

    const TotalCost cost = solMgr->cost;
    absl::Time start = absl::Now();
    absl::Duration task_timeout = timeout / num_actions;

    if (actions & RANDOM_WALK)
    {
      randomWalk(problem, *solMgr, task_timeout);
      num_actions--;
      absl::Duration rem = timeout - (start - absl::Now());
      task_timeout = rem / num_actions;
    }

    if (actions & ELIMINATE_EDGE)
    {
      for (int i = 0; i < eliminationCount; i++)
      {
        eliminateExpensiveEdge(problem, *solMgr, task_timeout / eliminationCount);
      }
      num_actions--;
      task_timeout = (timeout - (start - absl::Now())) / num_actions;
    }

    makeFeasible(problem, *solMgr, timeout / num_actions);
    if (!solMgr->isFeasible())
      return nullptr;

    if (actions & ONE_MAX)
    {
      makeCheap(problem, *solMgr, timeout / num_actions);
      num_actions--;
      absl::Duration rem = timeout - (start - absl::Now());
      task_timeout = rem / num_actions;
    }

    if (actions & TWO_MAX)
    {
      makeCheaper(problem, *solMgr, timeout / num_actions);
    }

    if (!solMgr->isFeasible() || solMgr->cost >= cost)
      return nullptr;

    return solMgr;
  }
}
