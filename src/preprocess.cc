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

#include <optional>
#include "preprocess.h"

namespace iopddl
{
  // finds a unique mapping between src and dst of edge
  std::optional<Mapping> getMapping(const Problem &problem, const Edge &edge)
  {
    Mapping mapping;
    for (StrategyIdx srcStrIdx = 0; srcStrIdx < edge.costs.size(); srcStrIdx++)
    {
      if (problem.nodes[edge.src].costs[srcStrIdx] < 1000000000000000000)
        for (StrategyIdx dstStrIdx = 0; dstStrIdx < edge.costs[srcStrIdx].size(); dstStrIdx++)
          if (edge.costs[srcStrIdx][dstStrIdx] + problem.nodes[edge.dst].costs[dstStrIdx] < 1000000000000000000)
            if (mapping.size() > srcStrIdx)
              // this is already the second non-expensive edge, the mapping is not unique
              return std::nullopt;
            else
              mapping.push_back(dstStrIdx);
      if (mapping.size() != srcStrIdx + 1)
        // this strategy has no non-expensive edge and therefore should not be used, choose any mapping
        mapping.push_back(0);
    }
    return mapping;
  }

  // calculates a pseudo-topological order of the nodes
  // more precisely, the first occurences of all SCCs will form a topological order of the SCC graph
  std::vector<NodeIdx> pseudoTopSort(Problem &problem)
  {
    // orders the nodes by their finishing time during DFS
    std::vector<NodeIdx> order;
    std::vector<bool> visited(problem.nodes.size(), false);
    std::function<void(NodeIdx)> dfs = [&](NodeIdx nodeIdx)
    {
      visited[nodeIdx] = true;
      for (const auto &[_, edgePtr] : problem.nodes[nodeIdx].edges)
        if (edgePtr->mapping)
          if (!visited[edgePtr->dst])
            dfs(edgePtr->dst);
      order.push_back(nodeIdx);
    };
    for (NodeIdx nodeIdx = 0; nodeIdx < problem.nodes.size(); ++nodeIdx)
      if (!visited[nodeIdx])
        dfs(nodeIdx);
    std::reverse(order.begin(), order.end());
    return order;
  }

  // merges all nodes with feasible mapping into this node
  void mergeInto(Problem &problem, NodeIdx src)
  {
    // Attention: We merge along edges src -> dst that have a mapping,
    // but we merge dst into src and not the other way around.
    // There is no guarantee that dst -> src also has a mapping.

    Node &node = problem.nodes[src];
    if (node.dependency != -1)
      // this node was already merged, skip it
      return;

    std::vector<EdgeIdx> dstKeys;
    for (const auto &[dst, _] : node.edges)
      dstKeys.push_back(dst);

    while (!dstKeys.empty())
    {
      NodeIdx dst = dstKeys.back();
      dstKeys.pop_back();

      if (node.edges.find(dst) == node.edges.end())
        // this edge was already handled in a previous iteration, skip it
        continue;

      Edge &edge = *node.edges.at(dst);
      if (!edge.mapping)
        // this edge has no mapping, skip it
        continue;

      Mapping &mapping = edge.mapping.value();

      Edge twin = *edge.twin.get();
      Node &other = problem.nodes[edge.dst];

      // mark dst as merged into src
      other.dependency = edge.src;
      other.mapping = mapping;

      // remove edge from src and twin from dst
      node.edges.erase(edge.dst);
      other.edges.erase(twin.dst);

      // add mapped dst costs to src costs
      for (StrategyIdx strIdx1 = 0; strIdx1 < node.costs.size(); strIdx1++)
      {
        StrategyIdx strIdx2 = mapping[strIdx1];
        node.costs[strIdx1] += other.costs[strIdx2];
        node.costs[strIdx1] += edge.costs[strIdx1][strIdx2];
      }

      // add intervals and mapped usages
      for (int i = 0; i < other.intervals.size(); i++)
      {
        node.intervals.push_back(other.intervals[i]);
        for (StrategyIdx strIdx1 = 0; strIdx1 < node.usages.size(); strIdx1++)
          node.usages[strIdx1].push_back(other.usages[mapping[strIdx1]][i]);
      }

      for (auto &entry : other.edges)
      {
        auto otherEdge = entry.second;
        auto otherTwin = otherEdge->twin;

        // remove other twin from its src
        problem.nodes[otherEdge->dst].edges.erase(otherEdge->src);

        if (node.edges.find(otherEdge->dst) == node.edges.end())
        {
          // rewire other edge and other twin
          otherEdge->src = edge.src;
          otherTwin->dst = twin.dst;

          // add other edge to node
          node.edges.emplace(otherEdge->dst, otherEdge);

          // update key of other twin
          problem.nodes[otherEdge->dst].edges.emplace(otherTwin->dst, otherTwin);

          // rearrange the costs of otherEdge
          std::vector<std::vector<TotalCost>> newCosts;
          for (StrategyIdx strIdx1 = 0; strIdx1 < edge.costs.size(); strIdx1++)
            newCosts.push_back(otherEdge->costs[mapping[strIdx1]]);
          otherEdge->costs = std::move(newCosts);
          otherEdge->mapping = getMapping(problem, *otherEdge);

          // rearrange the costs of otherTwin
          newCosts.clear();
          for (StrategyIdx strIdx1 = 0; strIdx1 < otherTwin->costs.size(); strIdx1++)
          {
            std::vector<TotalCost> temp;
            for (StrategyIdx strIdx2 = 0; strIdx2 < otherEdge->costs.size(); strIdx2++)
              temp.push_back(otherTwin->costs[strIdx1][mapping[strIdx2]]);
            newCosts.push_back(temp);
          }
          otherTwin->costs = std::move(newCosts);
          otherTwin->mapping = getMapping(problem, *otherTwin);
        }
        else
        {
          // merge parallel edges
          auto existingEdge = node.edges.at(otherEdge->dst);
          auto existingTwin = existingEdge->twin;

          // add the costs of other edge to existing edge
          for (StrategyIdx strIdx1 = 0; strIdx1 < existingEdge->costs.size(); strIdx1++)
            for (StrategyIdx strIdx2 = 0; strIdx2 < existingTwin->costs.size(); strIdx2++)
              existingEdge->costs[strIdx1][strIdx2] += otherEdge->costs[mapping[strIdx1]][strIdx2];
          existingEdge->mapping = getMapping(problem, *existingEdge);

          // add the costs of other twin to existing twin
          for (StrategyIdx strIdx1 = 0; strIdx1 < existingTwin->costs.size(); strIdx1++)
            for (StrategyIdx strIdx2 = 0; strIdx2 < existingEdge->costs.size(); strIdx2++)
              existingTwin->costs[strIdx1][strIdx2] += otherTwin->costs[strIdx1][mapping[strIdx2]];
          existingTwin->mapping = getMapping(problem, *existingTwin);
        }
        dstKeys.push_back(otherEdge->dst);
      }
    }
  }

  // preprocessing: merge nodes
  void preprocess(Problem &problem)
  {
    auto start_time = absl::Now();
    std::cout << "# Preprocessing: Merging nodes" << std::endl;

    // calculate mappings
    for (Node &node : problem.nodes)
      for (auto &[_, edgePtr] : node.edges)
        edgePtr->mapping = getMapping(problem, *edgePtr);

    // pseudo top-sort
    std::vector<NodeIdx> order = pseudoTopSort(problem);

    // merge nodes
    for (NodeIdx nodeIdx : order)
      mergeInto(problem, nodeIdx);

    // calculate edge count
    EdgeIdx oldEdgeCount = problem.edge_count;
    problem.edge_count = 0;
    for (Node &node : problem.nodes)
      if (node.dependency == -1)
        problem.edge_count += node.edges.size();
    problem.edge_count /= 2;

    // log the merge results
    int independentNodeCount = 0;
    for (const Node &node : problem.nodes)
      if (node.dependency == -1)
        independentNodeCount++;

    std::cout << "# Problem size reduced from " << problem.nodes.size() << " to " << independentNodeCount << " nodes and from " << oldEdgeCount << " to " << problem.edge_count << " edges." << std::endl;

    auto end_time = absl::Now();
    std::cout << "# Time: " << absl::ToDoubleSeconds(end_time - start_time) << "s" << std::endl;
  }
}
