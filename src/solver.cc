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

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "iopddl.h"
#include "phase.h"
#include "preprocess.h"
#include "search.h"
#include "solution.h"
#include <barrier>
#include <shared_mutex>
#include <thread>
#include <vector>

namespace iopddl
{
  const int num_threads = std::thread::hardware_concurrency();

  struct ArgData
  {
    Problem problem;
    int id;
    absl::Duration timeout;
  };

  int globalPhase = 0;

  int globalSolutionVersion = 1;
  std::shared_mutex bestSolutionLock;
  std::shared_ptr<SolutionManager> bestSolution;

  std::vector<std::vector<std::shared_ptr<SolutionManager>>> threadSolutions;

  // update the best found solution in a thread-safe way
  void updateBestSolution(std::shared_ptr<SolutionManager> sm)
  {
    bestSolutionLock.lock_shared();
    if (bestSolution == nullptr || bestSolution->cost > sm->cost)
    {
      bestSolution = sm;
      globalSolutionVersion++;
      std::cout << "# Solution improved to: " << bestSolution->cost << std::endl;
    }
    bestSolutionLock.unlock_shared();
  }

  void finish_phase() noexcept
  {
    PhaseConfig cfg = phases[globalPhase];

    // Collect all solutions
    std::vector<std::shared_ptr<SolutionManager>> solutions;
    for (auto &threadSolution : threadSolutions)
    {
      solutions.insert(solutions.end(), threadSolution.begin(), threadSolution.end());
      threadSolution.clear();
    }

    // Sort solutions by cost
    std::sort(solutions.begin(), solutions.end(), [&](auto a, auto b)
              { return a->cost < b->cost; });

    // Assign solutions to threads
    for (int i = 0; i < num_threads || i < std::min((int)solutions.size(), cfg.solution_count); i++)
      threadSolutions[i % num_threads].push_back(solutions[i % solutions.size()]);
  }

  std::barrier thread_sync(num_threads, finish_phase);

  void solver_thread(void *param)
  {
    ArgData *arg = (ArgData *)param;
    // INITIALISATION: find feasible solutions
    while (threadSolutions[arg->id].size() < phases[0].solution_count / num_threads && globalPhase < 1)
    {
      auto sm = findSolution(arg->problem, arg->timeout / 100);
      if (sm != nullptr)
      {
        threadSolutions[arg->id].emplace_back(sm);
        updateBestSolution(sm);
      }
    }
    // std::cout << "# Thread " << arg->id << " found " << threadSolutions[arg->id].size() << " feasible solutions" << std::endl;

    // PHASES
    for (int localPhase = 0; localPhase < num_phases; localPhase++)
    {
      PhaseConfig phaseCfg = phases[localPhase];
      if (phaseCfg.solution_count > 1)
      {
        // phase that works on multiple solutions

        std::vector<int> fails(threadSolutions[arg->id].size());
        for (int solIdx = 0; solIdx < threadSolutions[arg->id].size(); solIdx++)
          fails[solIdx] = 0;

        for (int solIdx = 0; globalPhase == localPhase; solIdx++, solIdx %= threadSolutions[arg->id].size())
        {
          auto sm = improveSolution(arg->problem, threadSolutions[arg->id][solIdx]->solution, phaseCfg.actions, fails[solIdx], arg->timeout / 100);
          if (sm != nullptr)
          {
            fails[solIdx] = 0;
            threadSolutions[arg->id][solIdx] = sm;
            updateBestSolution(sm);
          }
          else
          {
            fails[solIdx]++;
          }
        }
        // std::cout << "# Thread " << arg->id << " finished phase " << localPhase << " and is waiting for the next phase" << std::endl;
        if (phases[globalPhase].solution_count > 1)
          thread_sync.arrive_and_wait();
      }
      else
      {
        // phase that works on the best solution

        int localSolutionVersion = 0;
        int fails = 0;
        std::shared_ptr<SolutionManager> localSolution = nullptr;
        while (globalPhase == localPhase)
        {
          if (localSolutionVersion < globalSolutionVersion)
          {
            fails = 0;
            bestSolutionLock.lock_shared();
            localSolution = bestSolution;
            localSolutionVersion = globalSolutionVersion;
            bestSolutionLock.unlock_shared();
          }
          auto sm = improveSolution(arg->problem, localSolution->solution, phaseCfg.actions, fails, arg->timeout / 100);
          if (sm != nullptr)
          {
            updateBestSolution(sm);
          }
          else
          {
            fails++;
          }
        }
        // std::cout << "# Thread " << arg->id << " finished phase " << localPhase << " and is moving to the next phase" << std::endl;
      }
    }
  }

  absl::StatusOr<Solution> Solve(Problem &problem, absl::Duration timeout)
  {
    preprocess(problem);

    threadSolutions.resize(num_threads);
    std::thread *threads = new std::thread[num_threads];
    ArgData *args = new ArgData[num_threads];

    for (int i = 0; i < num_threads; i++)
    {
      args[i].id = i;
      args[i].problem = problem;
      args[i].timeout = timeout;
      threads[i] = std::thread(solver_thread, &args[i]);
    }
    std::cout << "# Searching for " << phases[0].solution_count << " initial solutions" << std::endl;

    absl::Duration exceeded;
    for (int i = 0; i < num_phases; i++)
    {
      absl::Time start = absl::Now();
      std::cout << "# Initiating phase " << i << ": " << phases[i] << std::endl;
      std::this_thread::sleep_for(absl::ToChronoMilliseconds(timeout - std::max(exceeded, absl::ZeroDuration())) * phases[i].timeout_factor);
      while (bestSolution == nullptr)
        std::this_thread::sleep_for(absl::ToChronoMilliseconds(timeout / 100));
      absl::Time end = absl::Now();
      exceeded = (phases[i].timeout_factor * timeout) - (end - start);
      // std::cout << "# Phase timeout exceeded by " << exceeded << std::endl;
      globalPhase++;
    }

    std::cout << "# Finalising solution" << std::endl;

    // Wait for all threads to finish
    for (int i = 0; i < num_threads; i++)
      threads[i].join();

    // We don't have to lock since all threads are joined
    auto sm = bestSolution;

    if (sm == nullptr)
      return absl::NotFoundError("# No solution found");

    sm->propagateSolution();
    return sm->solution;
  }
}
