#include "amr_dispatcher_core/workflow/payload_schema.hpp"

#include <algorithm>

namespace amr_dispatcher_core::workflow {

PayloadSchema& PayloadSchema::AddField(const std::string& key,
                                       const FieldConstraint& constraint) {
  fields_[key] = constraint;
  return *this;
}

PayloadValidationResult PayloadSchema::Validate(const OrderPayload& payload) const {
  PayloadValidationResult result;

  for (const auto& [key, constraint] : fields_) {
    const bool exists = payload.Has(key);
    if (!exists) {
      if (constraint.required) {
        result.valid = false;
        result.errors.push_back("missing required field: " + key);
      }
      continue;
    }

    // 字段类型及取值校验
    switch (constraint.type) {
      case PayloadFieldType::kString: {
        const auto val = payload.GetString(key);
        if (constraint.required && val.empty()) {
          result.valid = false;
          result.errors.push_back("field " + key + " cannot be empty");
        } else if (!constraint.allowed_values.empty()) {
          if (std::find(constraint.allowed_values.begin(), constraint.allowed_values.end(), val) ==
              constraint.allowed_values.end()) {
            result.valid = false;
            result.errors.push_back("field " + key + " has disallowed value: " + val);
          }
        }
        break;
      }
      case PayloadFieldType::kInt: {
        const int val = payload.GetInt(key);
        if (constraint.min_value && val < *constraint.min_value) {
          result.valid = false;
          result.errors.push_back("field " + key + " value " + std::to_string(val) +
                                  " is less than min " + std::to_string(*constraint.min_value));
        }
        if (constraint.max_value && val > *constraint.max_value) {
          result.valid = false;
          result.errors.push_back("field " + key + " value " + std::to_string(val) +
                                  " exceeds max " + std::to_string(*constraint.max_value));
        }
        break;
      }
      case PayloadFieldType::kDouble: {
        const double val = payload.GetDouble(key);
        if (constraint.min_value && val < *constraint.min_value) {
          result.valid = false;
          result.errors.push_back("field " + key + " value is less than min");
        }
        if (constraint.max_value && val > *constraint.max_value) {
          result.valid = false;
          result.errors.push_back("field " + key + " value exceeds max");
        }
        break;
      }
      case PayloadFieldType::kBool: {
        // bool 总是能被 GetBool 安全求值
        break;
      }
      case PayloadFieldType::kStringList: {
        const auto list = payload.GetStringList(key);
        if (constraint.required && list.empty()) {
          result.valid = false;
          result.errors.push_back("field " + key + " list cannot be empty");
        }
        break;
      }
    }
  }

  if (!result.valid) {
    result.message = "payload validation failed with " + std::to_string(result.errors.size()) +
                     " error(s): " + result.errors[0];
  } else {
    result.message = "payload validation passed";
  }

  return result;
}

PayloadSchema PayloadSchema::CreateTransportSchema() {
  PayloadSchema schema;
  FieldConstraint str_opt{PayloadFieldType::kString, false, {}, std::nullopt, std::nullopt};
  FieldConstraint dbl_opt{PayloadFieldType::kDouble, false, {}, std::nullopt, std::nullopt};
  schema.AddField("frame_id", str_opt)
      .AddField("pickup_x", dbl_opt)
      .AddField("pickup_y", dbl_opt)
      .AddField("dropoff_x", dbl_opt)
      .AddField("dropoff_y", dbl_opt);
  return schema;
}

PayloadSchema PayloadSchema::CreateStationTransportSchema() {
  PayloadSchema schema;
  FieldConstraint str_req{PayloadFieldType::kString, true, {}, std::nullopt, std::nullopt};
  schema.AddField("pickup_station", str_req)
      .AddField("dropoff_station", str_req);
  return schema;
}

PayloadSchema PayloadSchema::CreateStationOrderBatchSchema() {
  PayloadSchema schema;
  FieldConstraint str_opt{PayloadFieldType::kString, false, {}, std::nullopt, std::nullopt};
  FieldConstraint list_req{PayloadFieldType::kStringList, true, {}, std::nullopt, std::nullopt};
  FieldConstraint list_opt{PayloadFieldType::kStringList, false, {}, std::nullopt, std::nullopt};
  schema.AddField("batch_id", str_opt)
      .AddField("order_ids", list_req)
      .AddField("pickup_station_ids", list_opt)
      .AddField("dropoff_station_ids", list_opt);
  return schema;
}

}  // namespace amr_dispatcher_core::workflow
