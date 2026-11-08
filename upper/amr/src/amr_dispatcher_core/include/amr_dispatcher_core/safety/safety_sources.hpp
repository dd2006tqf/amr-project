#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "amr_dispatcher_core/safety/cmd_vel_gate.hpp"

namespace amr_dispatcher_core::safety {

struct SafetySourceDescriptor {
  std::string name;
  std::string description;
  std::function<SafetyEvent()> sample;
};

class SafetySourceRegistry {
 public:
  void Register(SafetySourceDescriptor descriptor);
  void Unregister(const std::string& name);

  std::vector<std::string> List() const;
  std::vector<SafetyEvent> Snapshot() const;
  bool Contains(const std::string& name) const;

 private:
  std::unordered_map<std::string, SafetySourceDescriptor> sources_;
};

}  // namespace amr_dispatcher_core::safety
