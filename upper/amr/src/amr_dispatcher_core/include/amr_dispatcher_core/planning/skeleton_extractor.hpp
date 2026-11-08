#ifndef AMR_DISPATCHER_CORE__PLANNING__SKELETON_EXTRACTOR_HPP_
#define AMR_DISPATCHER_CORE__PLANNING__SKELETON_EXTRACTOR_HPP_

#include <vector>

#include "amr_dispatcher_core/planning/map_geometry_utils.hpp"
#include "amr_dispatcher_core/planning/safety_space_filter.hpp"

namespace amr_dispatcher_core::planning {

// 经典 Zhang-Suen 细化算法：每轮执行两个子步，迭代消除满足条件的边界
// 像素，直至得到 1 像素宽的骨架线。
// 输入 BinaryGrid：1 = 可通行安全区域，0 = 障碍 / 禁止区域。
// 输出 BinaryGrid：仅保留 1 像素宽的骨架线（1 保留为 1，其余置 0）。
BinaryGrid ZhangSuenThinning(const BinaryGrid& grid);

// 受保护剪枝参数。
struct PruneConfig {
  double min_branch_length_m = 0.5;  // 剪除短于此长度的死胡同末端毛刺
  double resolution = 0.05;          // 米/像素
};

// 在骨架上执行受保护的毛刺剪枝：
//   1. 寻找度数为 1 的端点（8 邻域中只有 1 个相邻骨架像素）。
//   2. 从端点向内追踪分支，直至遇到分叉点（8 邻域连通组数 >= 3）或
//      另一个端点（连通组数 <= 1）。分支推进方向按与来向夹角最小者选取
//      （直线延续优先于斜向逃逸），且优先命中分叉候选——避免固定环序把
//      斜向紧邻的旁侧结构排在直线之前，导致追踪横向逃逸到相邻主线。
//   3. 分支长度（像素数 × resolution）短于 min_branch_length_m 时：
//        - 若分支上任意像素与 protected_points 中某点的欧式距离 < 1.5 像素，
//          视为承载站点的受保护分支，强制豁免，不可剪除。
//        - 否则将分支像素（不含分叉点）全部置 0；若停止点仅因该分支而
//          呈现为分叉（剩余邻居经 4 邻接连通后为单一连通组），则停止点
//          作为冗余凸起一并剪除。
//   4. 迭代重复，直至不再有可删除的未受保护短毛刺。
// 注：分叉判据采用 8 邻域环上的连通组数（而非裸像素数），避免把紧邻正交
// 交叉点的直线像素误判为分叉、在主干上残留 1 像素凸起伪影。
// 返回剪枝后的新栅格（入参只读，不原地修改）。
BinaryGrid PruneSkeletonProtected(
    const BinaryGrid& skeleton, const std::vector<PixelCoord>& protected_points,
    const PruneConfig& config);

}  // namespace amr_dispatcher_core::planning

#endif  // AMR_DISPATCHER_CORE__PLANNING__SKELETON_EXTRACTOR_HPP_
