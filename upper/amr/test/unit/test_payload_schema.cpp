#include "amr_dispatcher_core/workflow/payload_schema.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::workflow::FieldConstraint;
using amr_dispatcher_core::workflow::OrderPayload;
using amr_dispatcher_core::workflow::PayloadFieldType;
using amr_dispatcher_core::workflow::PayloadSchema;

TEST(PayloadSchemaTest, ValidStationTransportPayloadPasses) {
  auto schema = PayloadSchema::CreateStationTransportSchema();
  OrderPayload payload;
  payload.SetString("pickup_station", "receiving");
  payload.SetString("dropoff_station", "storage_a");

  auto result = schema.Validate(payload);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.errors.empty());
}

TEST(PayloadSchemaTest, MissingRequiredFieldFails) {
  auto schema = PayloadSchema::CreateStationTransportSchema();
  OrderPayload payload;
  payload.SetString("pickup_station", "receiving");
  // 缺少 dropoff_station

  auto result = schema.Validate(payload);
  EXPECT_FALSE(result.valid);
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_NE(result.errors[0].find("dropoff_station"), std::string::npos);
}

TEST(PayloadSchemaTest, AllowedValuesConstraint) {
  PayloadSchema schema;
  FieldConstraint constraint;
  constraint.type = PayloadFieldType::kString;
  constraint.required = true;
  constraint.allowed_values = {"north", "south"};
  schema.AddField("zone", constraint);

  OrderPayload valid_payload;
  valid_payload.SetString("zone", "north");
  EXPECT_TRUE(schema.Validate(valid_payload).valid);

  OrderPayload invalid_payload;
  invalid_payload.SetString("zone", "east");
  auto res = schema.Validate(invalid_payload);
  EXPECT_FALSE(res.valid);
  EXPECT_NE(res.errors[0].find("disallowed value"), std::string::npos);
}

TEST(PayloadSchemaTest, NumericBoundsValidation) {
  PayloadSchema schema;
  FieldConstraint constraint;
  constraint.type = PayloadFieldType::kInt;
  constraint.required = true;
  constraint.min_value = 1.0;
  constraint.max_value = 10.0;
  schema.AddField("priority", constraint);

  OrderPayload p1;
  p1.SetString("priority", "5");
  EXPECT_TRUE(schema.Validate(p1).valid);

  OrderPayload p2;
  p2.SetString("priority", "0");
  EXPECT_FALSE(schema.Validate(p2).valid);

  OrderPayload p3;
  p3.SetString("priority", "15");
  EXPECT_FALSE(schema.Validate(p3).valid);
}

TEST(PayloadSchemaTest, ListFieldValidation) {
  auto schema = PayloadSchema::CreateStationOrderBatchSchema();
  OrderPayload payload;
  payload.SetList("order_ids", {"ord_1", "ord_2"});
  EXPECT_TRUE(schema.Validate(payload).valid);

  OrderPayload empty_list_payload;
  empty_list_payload.SetList("order_ids", {});
  EXPECT_FALSE(schema.Validate(empty_list_payload).valid);
}
