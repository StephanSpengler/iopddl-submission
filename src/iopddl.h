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
 * https://github.com/google/iopddl/blob/main/iopddl.h                       *
 * (Apache License 2.0)                                                      *
 *                                                                           *
 * Significant modifications have been made.                                 *
 ****************************************************************************/

#ifndef IOPDDL_H
#define IOPDDL_H

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/numeric/int128.h"
#include "absl/status/statusor.h"

namespace iopddl
{

  using Cost = int64_t;
  using Usage = int64_t;
  using TimeIdx = int64_t;
  using NodeIdx = int64_t;
  using EdgeIdx = int64_t;
  using StrategyIdx = int64_t;
  using Interval = std::pair<TimeIdx, TimeIdx>;
  using Solution = std::vector<StrategyIdx>;
  using TotalUsage = absl::int128;
  using TotalCost = absl::int128;

  using Mapping = std::vector<StrategyIdx>;

  struct Edge
  {
    NodeIdx src;
    NodeIdx dst;
    std::vector<std::vector<TotalCost>> costs;
    std::shared_ptr<Edge> twin;
    std::optional<Mapping> mapping = std::nullopt;
  };

  struct Node
  {
    NodeIdx idx;

    std::vector<TotalCost> costs;
    std::vector<Interval> intervals;
    std::vector<std::vector<TotalUsage>> usages;
    std::unordered_map<NodeIdx, std::shared_ptr<Edge>> edges;

    NodeIdx dependency = -1;
    std::optional<Mapping> mapping = std::nullopt;

    std::unordered_map<TimeIdx, std::vector<TotalUsage>> cache;
  };

  struct Problem
  {
    std::string name;
    std::vector<Node> nodes;
    Usage usage_limit;
    TimeIdx max_time;
    EdgeIdx edge_count = 0;

    std::unordered_map<TimeIdx, std::vector<NodeIdx>> cache;
  };

  absl::StatusOr<Problem> ReadProblem(const std::string &filename);

}

#endif // IOPDDL_H
