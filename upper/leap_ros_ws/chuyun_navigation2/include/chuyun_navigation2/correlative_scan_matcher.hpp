#pragma once

#include <cstddef>
#include <vector>

#include "chuyun_navigation2/grid_map.hpp"
#include "chuyun_navigation2/types.hpp"

namespace chuyun_navigation2
{

struct ScanMatcherParameters
{
  double search_translation{0.20};
  double search_rotation{0.16};
  double coarse_translation_step{0.05};
  double coarse_rotation_step{0.04};
  double fine_translation_step{0.01};
  double fine_rotation_step{0.01};
  double likelihood_sigma{0.10};
  double minimum_score{0.32};
  double minimum_valid_fraction{0.45};
  double prior_weight{0.025};
  double correction_gain{0.35};
  int minimum_points{25};
  int maximum_points{160};
};

struct ScanMatchResult
{
  bool accepted{false};
  Pose2D matched_pose;
  Pose2D corrected_pose;
  double score{0.0};
  double valid_fraction{0.0};
  double position_variance{0.25};
  double yaw_variance{0.10};
  size_t points_used{0};
};

// A small, allocation-light correlative matcher. It evaluates a coarse window
// around the odometry prediction and then refines the best pose locally.
class CorrelativeScanMatcher
{
public:
  CorrelativeScanMatcher(const GridMap & map, ScanMatcherParameters parameters);
  ScanMatchResult match(
    const Pose2D & predicted_pose,
    const std::vector<Point2D> & points_in_base_frame) const;

private:
  struct Score
  {
    double likelihood{-1.0};
    double objective{-1.0};
    double valid_fraction{0.0};
  };

  Score scorePose(
    const Pose2D & candidate,
    const Pose2D & predicted,
    const std::vector<Point2D> & points) const;

  const GridMap & map_;
  ScanMatcherParameters parameters_;
};

}  // namespace chuyun_navigation2
