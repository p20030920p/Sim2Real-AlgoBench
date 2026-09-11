// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT
//
// Name -> algorithm factory registry.
//
// This is what makes "change one index and the algorithm changes" work: every
// algorithm self-registers under a stable name, and callers instantiate by
// name. Adding an algorithm means adding one file and one ALGO_CORE_REGISTER
// line; nothing else in the tree changes.

#ifndef ALGO_CORE__REGISTRY_HPP_
#define ALGO_CORE__REGISTRY_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "algo_core/grid_planner.hpp"

namespace algo_core
{

class Registry
{
public:
  using Factory = std::function<GridPlanner::Ptr()>;

  /// The one and only registry.
  ///
  /// Defined out of line in registry.cpp on purpose. An inline function-local
  /// static gets emitted as a weak symbol in every translation unit that uses
  /// it, and the linker is then free to satisfy calls from a module's own copy.
  /// That produces two registries — one populated by the self-registering
  /// algorithms inside the library, one empty inside the caller — which is
  /// exactly the bug that makes a plugin report zero available algorithms.
  static Registry & instance();

  void add(const std::string & name, Factory factory)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & entry : entries_) {
      if (entry.name == name) {
        entry.factory = std::move(factory);
        return;
      }
    }
    entries_.push_back(Entry{name, std::move(factory)});
  }

  bool contains(const std::string & name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : entries_) {
      if (entry.name == name) {return true;}
    }
    return false;
  }

  /// Returns nullptr when the name is unknown. Callers should print
  /// availableNames() in that case rather than failing silently.
  GridPlanner::Ptr create(const std::string & name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : entries_) {
      if (entry.name == name) {return entry.factory();}
    }
    return nullptr;
  }

  std::vector<std::string> availableNames() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(entries_.size());
    for (const auto & entry : entries_) {names.push_back(entry.name);}
    return names;
  }

private:
  struct Entry
  {
    std::string name;
    Factory factory;
  };

  Registry() = default;
  mutable std::mutex mutex_;
  std::vector<Entry> entries_;
};

/// Self-registration helper. Instantiate once per algorithm in its .cpp file.
template<typename T>
struct Registrar
{
  explicit Registrar(const std::string & name)
  {
    Registry::instance().add(name, []() -> GridPlanner::Ptr {
      return std::make_unique<T>();
    });
  }
};

}  // namespace algo_core

/// Register an algorithm type under a name, at static-init time.
///
/// The generated variable is keyed on __LINE__ rather than on the type, because
/// the type is normally qualified (`ns::Type`) and `::` cannot be pasted into
/// an identifier. Two levels of indirection are needed so that __LINE__ is
/// expanded before the paste.
#define ALGO_CORE_DETAIL_CONCAT_IMPL(a, b) a##b
#define ALGO_CORE_DETAIL_CONCAT(a, b) ALGO_CORE_DETAIL_CONCAT_IMPL(a, b)

#define ALGO_CORE_REGISTER(TYPE, NAME) \
  namespace { \
  const ::algo_core::Registrar<TYPE> \
  ALGO_CORE_DETAIL_CONCAT(g_algo_registrar_line_, __LINE__)(NAME); \
  }  // namespace

#endif  // ALGO_CORE__REGISTRY_HPP_
