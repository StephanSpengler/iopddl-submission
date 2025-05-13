# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.25.1)
project(iopddl)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(external/abseil-cpp)
add_subdirectory(external/nlohmann_json)
add_executable(iopddl
  src/iopddl.cc
  src/main.cc
  src/preprocess.cc
  src/SegmentTree.cc
  src/solution.cc
  src/solver.cc
  src/search.cc
)
target_link_libraries(iopddl -static
  absl::numeric
  absl::statusor
  absl::strings
  nlohmann_json::nlohmann_json
)
