#include "amr_dispatcher_core/safety/safety_sources.hpp"

#include <utility>

namespace amr_dispatcher_core::safety {

void SafetySourceRegistry::Register(SafetySourceDescriptor descriptor) {
  sources_[descriptor.name] = std::move(descriptor);
}

void SafetySourceRegistry::Unregister(const std::string& name) {
  sources_.erase(name);
}

std::vector<std::string> SafetySourceRegistry::List() const {
  std::vector<std::string> names;
  names.reserve(sources_.size());
  for (const auto& [name, _] : sources_) {
    names.push_back(name);
  }
  return names;
}

std::vector<SafetyEvent> SafetySourceRegistry::Snapshot() const {
  std::vector<SafetyEvent> events;
  for (const auto& [name, desc] : sources_) {
    if (desc.sample) {
      auto e = desc.sample();
      if (e.source.empty()) e.source = name;
      events.push_back(std::move(e));
    }
  }
  return events;
}

bool SafetySourceRegistry::Contains(const std::string& name) const {
  return sources_.find(name) != sources_.end();
}

}  // namespace amr_dispatcher_core::safety
