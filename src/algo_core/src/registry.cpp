// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#include "algo_core/registry.hpp"

namespace algo_core
{

Registry & Registry::instance()
{
  static Registry registry;
  return registry;
}

}  // namespace algo_core
