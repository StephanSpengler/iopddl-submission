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
 * https://github.com/google/iopddl/blob/main/main.h                         *
 * (Apache License 2.0)                                                      *
 *                                                                           *
 * Minor modifications have been made.                                       *
 ****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <string>

#include "absl/strings/str_join.h"
#include "absl/time/time.h"
#include "iopddl.h"
#include "solver.h"

int main(int argc, char *argv[])
{
  const std::string filename = argv[1];
  const absl::Duration timeout = absl::Seconds(atoi(argv[2]));
  auto problem = iopddl::ReadProblem(filename);
  if (!problem.ok())
    exit(1);
  const auto solution = iopddl::Solve(*problem, timeout);
  const auto result = solution.value_or(iopddl::Solution{});
  std::cout << "[" << absl::StrJoin(result, ", ") << "]" << std::endl;
  return 0;
}
