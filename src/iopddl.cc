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

/*****************************************************************************
 * This file is based on:                                                    *
 * https://github.com/google/iopddl/blob/main/iopddl.cc                      *
 * (Apache License 2.0)                                                      *
 *                                                                           *
 * Significant modifications have been made.                                 *
 ****************************************************************************/

#include "iopddl.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "nlohmann/json.hpp"

namespace iopddl
{
  absl::StatusOr<Problem> ReadProblem(const std::string &filename)
  {
    // read problem from json and create Problem struct
    const nlohmann::json data = nlohmann::json::parse(std::ifstream(filename));
    Problem problem = {.name = data["problem"]["name"]};

    // create node structs with intervals
    const auto &nodes = data["problem"]["nodes"];
    for (const auto &node_interval : nodes["intervals"])
    {
      problem.nodes.push_back({.intervals = {{node_interval[0], node_interval[1]}}});
      problem.max_time = std::max(problem.max_time, static_cast<TimeIdx>(node_interval[1]));
    }

    // add index, costs and usages to nodes
    for (NodeIdx node_idx = 0; node_idx < problem.nodes.size(); ++node_idx)
    {
      Node &node = problem.nodes[node_idx];
      node.idx = node_idx;

      const auto &costs = nodes["costs"][node_idx];
      node.costs.reserve(costs.size());
      for (StrategyIdx strategy_idx = 0; strategy_idx < costs.size(); ++strategy_idx)
        node.costs.push_back(static_cast<Cost>(costs[strategy_idx]));

      const auto &usages = nodes["usages"][node_idx];
      node.usages.reserve(usages.size());
      for (StrategyIdx strategy_idx = 0; strategy_idx < usages.size(); ++strategy_idx)
        node.usages.push_back({static_cast<Usage>(usages[strategy_idx])});
    }

    // add edges to problem
    const auto &edges = data["problem"]["edges"];
    problem.edge_count = edges["nodes"].size();
    for (EdgeIdx edge_idx = 0; edge_idx < edges["nodes"].size(); ++edge_idx)
    {
      const auto nodeIdx1 = edges["nodes"][edge_idx][0];
      const auto nodeIdx2 = edges["nodes"][edge_idx][1];

      auto &node1 = problem.nodes[nodeIdx1];
      auto &node2 = problem.nodes[nodeIdx2];

      if (node1.edges.find(nodeIdx2) == node1.edges.end())
      {
        // edge does not exist, create it

        std::vector<std::vector<TotalCost>> costs12(node1.costs.size());
        std::vector<std::vector<TotalCost>> costs21(node2.costs.size());
        const auto costs = edges["costs"][edge_idx];
        for (StrategyIdx strIdx = 0; strIdx < costs.size(); ++strIdx)
        {
          costs12[strIdx / node2.costs.size()].push_back(static_cast<Cost>(costs[strIdx]));
          costs21[strIdx % node2.costs.size()].push_back(static_cast<Cost>(costs[strIdx]));
        }

        auto edge12 = std::make_shared<Edge>(nodeIdx1, nodeIdx2, costs12);
        auto edge21 = std::make_shared<Edge>(nodeIdx2, nodeIdx1, costs21);

        edge12->twin = edge21;
        edge21->twin = edge12;

        node1.edges.emplace(nodeIdx2, std::move(edge12));
        node2.edges.emplace(nodeIdx1, std::move(edge21));
      }
      else
      {
        // edge already exists, merge it

        auto edge12 = node1.edges[nodeIdx2];
        auto edge21 = node2.edges[nodeIdx1];

        const auto costs = edges["costs"][edge_idx];
        for (StrategyIdx strIdx = 0; strIdx < costs.size(); ++strIdx)
        {
          StrategyIdx strIdx1 = strIdx / node2.costs.size();
          StrategyIdx strIdx2 = strIdx % node2.costs.size();

          edge12->costs[strIdx1][strIdx2] += static_cast<Cost>(costs[strIdx]);
          edge21->costs[strIdx2][strIdx1] += static_cast<Cost>(costs[strIdx]);
        }
      }
    }

    // set usage limit
    if (data["problem"].contains("usage_limit"))
    {
      problem.usage_limit = data["problem"]["usage_limit"];
    }
    else
    {
      problem.usage_limit = std::numeric_limits<Usage>::max();
      std::cout << "No usage limit specified, using default value of " << std::numeric_limits<Usage>::max() << std::endl;
    }
    return problem;
  }
}
