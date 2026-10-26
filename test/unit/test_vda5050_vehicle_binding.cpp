#include "amr_dispatcher_core/fleet/vda5050_vehicle_binding.hpp"

#include <gtest/gtest.h>

using namespace amr_dispatcher_core::fleet;

namespace {

Vda5050VehicleBinding Bind(const std::string& assigned, const std::vector<std::string>& managed,
                           std::size_t* cursor) {
  Vda5050VehicleBindingRequest request;
  request.assigned_vehicle_id = assigned;
  request.managed_vehicle_ids = &managed;
  request.round_robin_cursor = cursor;
  return BindVda5050Vehicle(request);
}

}  // namespace

TEST(Vda5050VehicleBindingTest, ExplicitAssignmentWins) {
  const std::vector<std::string> managed{"amr_01", "amr_02"};
  std::size_t cursor = 0;
  const auto binding = Bind("amr_02", managed, &cursor);
  EXPECT_EQ(binding.vehicle_id, "amr_02");
  EXPECT_TRUE(binding.from_assignment);
  EXPECT_EQ(cursor, 0u);  // 显式绑定不推进游标
}

// 配置改动后清单里已没有这台车：必须回退，而不是把订单发给不存在的车。
TEST(Vda5050VehicleBindingTest, UnknownAssignmentFallsBackToRoundRobin) {
  const std::vector<std::string> managed{"amr_01", "amr_02"};
  std::size_t cursor = 0;
  const auto binding = Bind("amr_99", managed, &cursor);
  EXPECT_EQ(binding.vehicle_id, "amr_01");
  EXPECT_FALSE(binding.from_assignment);
  EXPECT_EQ(cursor, 1u);
}

TEST(Vda5050VehicleBindingTest, EmptyAssignmentRoundRobinsInOrder) {
  const std::vector<std::string> managed{"amr_01", "amr_02", "amr_03"};
  std::size_t cursor = 0;
  EXPECT_EQ(Bind("", managed, &cursor).vehicle_id, "amr_01");
  EXPECT_EQ(Bind("", managed, &cursor).vehicle_id, "amr_02");
  EXPECT_EQ(Bind("", managed, &cursor).vehicle_id, "amr_03");
  EXPECT_EQ(Bind("", managed, &cursor).vehicle_id, "amr_01");  // 环绕
  EXPECT_EQ(cursor, 1u);
}

// reconfigure 后清单变短（游标可能越界）：取模回绕，不能算出野下标。
TEST(Vda5050VehicleBindingTest, CursorBeyondListLengthWraps) {
  const std::vector<std::string> managed{"amr_01", "amr_02"};
  std::size_t cursor = 7;
  const auto binding = Bind("", managed, &cursor);
  EXPECT_EQ(binding.vehicle_id, "amr_02");  // 7 % 2 == 1
  EXPECT_EQ(cursor, 0u);
}

TEST(Vda5050VehicleBindingTest, SingleVehicleIsStable) {
  const std::vector<std::string> managed{"amr_01"};
  std::size_t cursor = 0;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(Bind("", managed, &cursor).vehicle_id, "amr_01");
    EXPECT_EQ(cursor, 0u);
  }
}

TEST(Vda5050VehicleBindingTest, NoManagedVehiclesMeansNoBinding) {
  const std::vector<std::string> managed;
  std::size_t cursor = 0;
  const auto binding = Bind("amr_01", managed, &cursor);
  EXPECT_TRUE(binding.vehicle_id.empty());
  EXPECT_FALSE(binding.from_assignment);
  EXPECT_EQ(cursor, 0u);
}

TEST(Vda5050VehicleBindingTest, NullListMeansNoBinding) {
  Vda5050VehicleBindingRequest request;
  request.assigned_vehicle_id = "amr_01";
  const auto binding = BindVda5050Vehicle(request);
  EXPECT_TRUE(binding.vehicle_id.empty());
}

// 游标指针为空是允许的（只问"该绑谁"、不动状态）。
TEST(Vda5050VehicleBindingTest, NullCursorIsTolerated) {
  const std::vector<std::string> managed{"amr_01", "amr_02"};
  EXPECT_EQ(Bind("", managed, nullptr).vehicle_id, "amr_01");
  EXPECT_EQ(Bind("amr_02", managed, nullptr).vehicle_id, "amr_02");
}

TEST(Vda5050VehicleBindingTest, IsManagedChecksMembership) {
  const std::vector<std::string> managed{"amr_01", "amr_02"};
  EXPECT_TRUE(IsManagedVda5050Vehicle(managed, "amr_01"));
  EXPECT_FALSE(IsManagedVda5050Vehicle(managed, "amr_03"));
  EXPECT_FALSE(IsManagedVda5050Vehicle(managed, ""));  // 空 id 永不算受管
  EXPECT_FALSE(IsManagedVda5050Vehicle({}, "amr_01"));
}
