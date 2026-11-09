// 设施目录（facilities.yaml）的单测。
//
// 覆盖点：
//   1. 目录解析：resources[] 的 id / type / station_id / enabled / exclusive
//   2. 目录校验：重复 id、空 type、空 id
//   3. FindFacilityResource / FindFirstAvailableResource 的选取语义

#include "amr_dispatcher_core/facility/facility_catalog.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

using namespace amr_dispatcher_core::facility;

namespace {

class FacilityCatalogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("facility_catalog_test_" +
            std::to_string(reinterpret_cast<std::uintptr_t>(this)));
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  std::filesystem::path WriteFile(const std::string& name, const std::string& content) {
    const auto path = dir_ / name;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
  }

  std::filesystem::path dir_;
};

}  // namespace

TEST_F(FacilityCatalogTest, LoadFacilityCatalog_ParsesAllFields) {
  const auto path = WriteFile("facilities.yaml", R"(resources:
  - id: receiving_door
    type: door
    station_id: receiving
    enabled: true
    exclusive: true
  - id: freight_elevator
    type: elevator
    station_id: packing
    enabled: false
    exclusive: false
)");

  const auto catalog = LoadFacilityCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->resources.size(), 2u);
  const auto* door = FindFacilityResource(*catalog, "receiving_door");
  ASSERT_NE(door, nullptr);
  EXPECT_EQ(door->type, "door");
  EXPECT_EQ(door->station_id, "receiving");
  EXPECT_TRUE(door->enabled);
  EXPECT_TRUE(door->exclusive);
  const auto* elevator = FindFacilityResource(*catalog, "freight_elevator");
  ASSERT_NE(elevator, nullptr);
  EXPECT_FALSE(elevator->enabled);
  EXPECT_FALSE(elevator->exclusive);
  EXPECT_EQ(FindFacilityResource(*catalog, "nope"), nullptr);
}

TEST_F(FacilityCatalogTest, LoadFacilityCatalog_DuplicateIdIsRejected) {
  const auto path = WriteFile("facilities.yaml", R"(resources:
  - id: charger_main
    type: charger
  - id: charger_main
    type: charger
)");

  EXPECT_FALSE(LoadFacilityCatalog(path).has_value());
}

TEST_F(FacilityCatalogTest, LoadFacilityCatalog_MissingTypeIsRejected) {
  const auto path = WriteFile("facilities.yaml", R"(resources:
  - id: charger_main
    station_id: dock
)");

  // type 缺失 -> 记录不完整 -> 整份目录判负
  EXPECT_FALSE(LoadFacilityCatalog(path).has_value());
}

TEST_F(FacilityCatalogTest, LoadFacilityCatalog_MissingFileReturnsNullopt) {
  EXPECT_FALSE(LoadFacilityCatalog(dir_ / "does_not_exist.yaml").has_value());
}

TEST_F(FacilityCatalogTest, ValidateFacilityCatalog_RejectsEmptyId) {
  FacilityCatalog catalog;
  catalog.resources.push_back(FacilityResource{"", "door", "receiving", true, true});
  std::string message;
  EXPECT_FALSE(ValidateFacilityCatalog(catalog, &message));
  EXPECT_FALSE(message.empty());
}

TEST_F(FacilityCatalogTest, FindFirstAvailableResource_SkipsDisabledAndUnavailable) {
  FacilityCatalog catalog;
  catalog.resources.push_back(FacilityResource{"charger_a", "charger", "dock", true, true});
  catalog.resources.push_back(FacilityResource{"charger_b", "charger", "dock", false, true});
  catalog.resources.push_back(FacilityResource{"charger_c", "charger", "dock", true, true});

  const auto* resource = FindFirstAvailableResource(catalog, "charger", {"charger_a"});

  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->id, "charger_c");
}

TEST_F(FacilityCatalogTest, FindFirstAvailableResource_TypeMismatchReturnsNull) {
  FacilityCatalog catalog;
  catalog.resources.push_back(FacilityResource{"receiving_door", "door", "receiving", true, true});

  EXPECT_EQ(FindFirstAvailableResource(catalog, "elevator", {}), nullptr);
}
