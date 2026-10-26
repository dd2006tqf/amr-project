#include "amr_dispatcher_core/workflow/order_batch_bundling.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::workflow::OrderBatchBundleRequest;
using amr_dispatcher_core::workflow::OrderBatchBundleResult;
using amr_dispatcher_core::workflow::OrderBatchItem;
using amr_dispatcher_core::workflow::PlanOrderBatchBundling;

TEST(OrderBatchBundlingTest, EmptyBatchIdFails) {
  OrderBatchBundleRequest req;
  req.batch_id = "";
  auto res = PlanOrderBatchBundling(req);
  EXPECT_FALSE(res.success);
}

TEST(OrderBatchBundlingTest, EmptyItemsSucceedsWithNoGroups) {
  OrderBatchBundleRequest req;
  req.batch_id = "b1";
  auto res = PlanOrderBatchBundling(req);
  EXPECT_TRUE(res.success);
  EXPECT_TRUE(res.groups.empty());
}

TEST(OrderBatchBundlingTest, InvalidOrderItemsRejected) {
  OrderBatchBundleRequest req;
  req.batch_id = "b1";
  req.items = {{"", "A", "B", 1, 0}};
  EXPECT_FALSE(PlanOrderBatchBundling(req).success);

  req.items = {{"o1", "", "B", 1, 0}};
  EXPECT_FALSE(PlanOrderBatchBundling(req).success);

  req.items = {{"o1", "A", "A", 1, 0}};
  EXPECT_FALSE(PlanOrderBatchBundling(req).success);
}

TEST(OrderBatchBundlingTest, ExactMatchOrdersBundledIntoSingleGroup) {
  OrderBatchBundleRequest req;
  req.batch_id = "batch_test";
  req.items = {
      {"ord_1", "receiving", "storage_a", 1, 1000},
      {"ord_2", "receiving", "storage_a", 5, 500},
  };
  auto res = PlanOrderBatchBundling(req);
  ASSERT_TRUE(res.success);
  ASSERT_EQ(res.groups.size(), 1u);
  const auto& grp = res.groups[0];
  EXPECT_EQ(grp.priority, 5);
  EXPECT_EQ(grp.earliest_deadline_unix_ms, 500);
  EXPECT_EQ(grp.order_ids.size(), 2u);
  EXPECT_EQ(grp.station_sequence, (std::vector<std::string>{"receiving", "storage_a"}));
  EXPECT_EQ(grp.legs.size(), 1u);
  EXPECT_EQ(grp.legs[0].bundled_order_ids.size(), 2u);
}

TEST(OrderBatchBundlingTest, RouteChainingMergesConnectingLegs) {
  OrderBatchBundleRequest req;
  req.batch_id = "chain_test";
  req.enable_route_chaining = true;
  req.items = {
      {"ord_1", "receiving", "storage_a", 2, 1000},
      {"ord_2", "storage_a", "packing", 4, 800},
  };
  auto res = PlanOrderBatchBundling(req);
  ASSERT_TRUE(res.success);
  ASSERT_EQ(res.groups.size(), 1u);
  const auto& grp = res.groups[0];
  EXPECT_EQ(grp.priority, 4);
  EXPECT_EQ(grp.earliest_deadline_unix_ms, 800);
  EXPECT_EQ(grp.station_sequence, (std::vector<std::string>{"receiving", "storage_a", "packing"}));
  EXPECT_EQ(grp.legs.size(), 2u);
}

TEST(OrderBatchBundlingTest, RouteChainingDisabledKeepsSeparate) {
  OrderBatchBundleRequest req;
  req.batch_id = "no_chain_test";
  req.enable_route_chaining = false;
  req.items = {
      {"ord_1", "receiving", "storage_a", 2, 1000},
      {"ord_2", "storage_a", "packing", 4, 800},
  };
  auto res = PlanOrderBatchBundling(req);
  ASSERT_TRUE(res.success);
  EXPECT_EQ(res.groups.size(), 2u);
}
