#include "chuyun_navigation2/correlative_scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace chuyun_navigation2
{

CorrelativeScanMatcher::CorrelativeScanMatcher(
  const GridMap & map,
  ScanMatcherParameters parameters)
: map_(map), parameters_(parameters)
{
  parameters_.coarse_translation_step = std::max(0.005, parameters_.coarse_translation_step);
  parameters_.coarse_rotation_step = std::max(0.002, parameters_.coarse_rotation_step);
  parameters_.fine_translation_step = clamp(
    parameters_.fine_translation_step, 0.002, parameters_.coarse_translation_step);
  parameters_.fine_rotation_step = clamp(
    parameters_.fine_rotation_step, 0.001, parameters_.coarse_rotation_step);
  parameters_.likelihood_sigma = std::max(0.01, parameters_.likelihood_sigma);
  parameters_.correction_gain = clamp(parameters_.correction_gain, 0.0, 1.0);
  parameters_.maximum_points = std::max(parameters_.minimum_points, parameters_.maximum_points);
}

CorrelativeScanMatcher::Score CorrelativeScanMatcher::scorePose(
  const Pose2D & candidate,
  const Pose2D & predicted,
  const std::vector<Point2D> & points) const
{
  const double c = std::cos(candidate.yaw);
  const double s = std::sin(candidate.yaw);
  const double inverse_two_sigma_squared =
    0.5 / (parameters_.likelihood_sigma * parameters_.likelihood_sigma);
  double likelihood_sum = 0.0;
  size_t valid = 0;
  for (const Point2D & point : points) {
    const double world_x = candidate.x + c * point.x - s * point.y;
    const double world_y = candidate.y + s * point.x + c * point.y;
    double distance = 0.0;
    if (!map_.distanceAtWorld(world_x, world_y, distance)) continue;
    likelihood_sum += std::exp(-distance * distance * inverse_two_sigma_squared);
    ++valid;
  }

  Score score;
  if (valid < static_cast<size_t>(parameters_.minimum_points)) return score;
  score.valid_fraction = static_cast<double>(valid) / points.size();
  if (score.valid_fraction < parameters_.minimum_valid_fraction) return score;
  score.likelihood = likelihood_sum / valid;

  const double translation_scale = std::max(0.01, parameters_.search_translation);
  const double rotation_scale = std::max(0.01, parameters_.search_rotation);
  const double dx = (candidate.x - predicted.x) / translation_scale;
  const double dy = (candidate.y - predicted.y) / translation_scale;
  const double dyaw = normalizeAngle(candidate.yaw - predicted.yaw) / rotation_scale;
  const double prior_penalty = parameters_.prior_weight * (dx * dx + dy * dy + dyaw * dyaw);

  // Coverage prevents poses near the map boundary from scoring well using only a few beams.
  const double coverage_weight = 0.5 + 0.5 * score.valid_fraction;
  score.objective = score.likelihood * coverage_weight - prior_penalty;
  return score;
}

ScanMatchResult CorrelativeScanMatcher::match(
  const Pose2D & predicted_pose,
  const std::vector<Point2D> & points_in_base_frame) const
{
  ScanMatchResult result;
  result.matched_pose = predicted_pose;
  result.corrected_pose = predicted_pose;
  if (points_in_base_frame.size() < static_cast<size_t>(parameters_.minimum_points)) return result;

  std::vector<Point2D> points;
  if (points_in_base_frame.size() <= static_cast<size_t>(parameters_.maximum_points)) {
    points = points_in_base_frame;
  } else {
    points.reserve(parameters_.maximum_points);
    const double stride = static_cast<double>(points_in_base_frame.size()) / parameters_.maximum_points;
    for (int i = 0; i < parameters_.maximum_points; ++i) {
      const size_t index = std::min(
        points_in_base_frame.size() - 1,
        static_cast<size_t>(std::floor(i * stride)));
      points.push_back(points_in_base_frame[index]);
    }
  }
  result.points_used = points.size();

  Pose2D best_pose = predicted_pose;
  Score best_score = scorePose(best_pose, predicted_pose, points);
  const int translation_steps = std::max(
    0, static_cast<int>(std::ceil(
      parameters_.search_translation / parameters_.coarse_translation_step)));
  const int rotation_steps = std::max(
    0, static_cast<int>(std::ceil(
      parameters_.search_rotation / parameters_.coarse_rotation_step)));

  for (int yaw_step = -rotation_steps; yaw_step <= rotation_steps; ++yaw_step) {
    const double yaw = normalizeAngle(
      predicted_pose.yaw + yaw_step * parameters_.coarse_rotation_step);
    for (int y_step = -translation_steps; y_step <= translation_steps; ++y_step) {
      for (int x_step = -translation_steps; x_step <= translation_steps; ++x_step) {
        const Pose2D candidate{
          predicted_pose.x + x_step * parameters_.coarse_translation_step,
          predicted_pose.y + y_step * parameters_.coarse_translation_step,
          yaw};
        const Score score = scorePose(candidate, predicted_pose, points);
        if (score.objective > best_score.objective) {
          best_pose = candidate;
          best_score = score;
        }
      }
    }
  }

  // Refine with small 3x3x3 neighborhoods. This retains the accuracy of a dense
  // search while evaluating far fewer poses on low-power computers.
  double translation_step = parameters_.coarse_translation_step * 0.5;
  double rotation_step = parameters_.coarse_rotation_step * 0.5;
  while (translation_step >= parameters_.fine_translation_step ||
    rotation_step >= parameters_.fine_rotation_step)
  {
    translation_step = std::max(translation_step, parameters_.fine_translation_step);
    rotation_step = std::max(rotation_step, parameters_.fine_rotation_step);
    Pose2D refinement_center = best_pose;
    for (int yaw_offset = -1; yaw_offset <= 1; ++yaw_offset) {
      for (int y_offset = -1; y_offset <= 1; ++y_offset) {
        for (int x_offset = -1; x_offset <= 1; ++x_offset) {
          const Pose2D candidate{
            refinement_center.x + x_offset * translation_step,
            refinement_center.y + y_offset * translation_step,
            normalizeAngle(refinement_center.yaw + yaw_offset * rotation_step)};
          if (std::hypot(candidate.x - predicted_pose.x, candidate.y - predicted_pose.y) >
            parameters_.search_translation * 1.5)
          {
            continue;
          }
          if (std::abs(normalizeAngle(candidate.yaw - predicted_pose.yaw)) >
            parameters_.search_rotation * 1.5)
          {
            continue;
          }
          const Score score = scorePose(candidate, predicted_pose, points);
          if (score.objective > best_score.objective) {
            best_pose = candidate;
            best_score = score;
          }
        }
      }
    }
    if (translation_step <= parameters_.fine_translation_step &&
      rotation_step <= parameters_.fine_rotation_step)
    {
      break;
    }
    translation_step = std::max(parameters_.fine_translation_step, translation_step * 0.5);
    rotation_step = std::max(parameters_.fine_rotation_step, rotation_step * 0.5);
  }

  if (best_score.likelihood < parameters_.minimum_score ||
    best_score.valid_fraction < parameters_.minimum_valid_fraction)
  {
    result.score = std::max(0.0, best_score.likelihood);
    result.valid_fraction = best_score.valid_fraction;
    return result;
  }

  result.accepted = true;
  result.matched_pose = best_pose;
  result.corrected_pose = interpolate(predicted_pose, best_pose, parameters_.correction_gain);
  result.score = best_score.likelihood;
  result.valid_fraction = best_score.valid_fraction;
  result.position_variance = clamp(
    0.005 + 0.12 * (1.0 - result.score), 0.005, 0.20);
  result.yaw_variance = clamp(
    0.003 + 0.08 * (1.0 - result.score), 0.003, 0.12);
  return result;
}

}  // namespace chuyun_navigation2
