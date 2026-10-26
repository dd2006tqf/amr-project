// 订单载荷（OrderPayload）结构化校验 Schema。
//
// 背景：过去 OrderPayload 是非类型化的字符串键值对与列表字典，
// 上游发送的 JSON 或 KV 缺乏字段类型与必填约束校验，导致非法字段穿透进入派发链路。
//
// 本模块定义轻量、零依赖的 Schema 校验机制：
// 1. FieldType：指定字段类型要求（String, Int, Double, Bool, List）。
// 2. FieldConstraint：定义字段必填性 (required) 及允许取值或范围检查。
// 3. PayloadSchema：容纳多字段校验规则，对 OrderPayload 进行严格校验，
//    并输出明确可读的错误信息，同时可复用于 ValidateSiteConfig 与订单准入阶段。
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "amr_dispatcher_core/workflow/order_payload.hpp"

namespace amr_dispatcher_core::workflow {

enum class PayloadFieldType {
  kString,
  kInt,
  kDouble,
  kBool,
  kStringList,
};

struct FieldConstraint {
  PayloadFieldType type = PayloadFieldType::kString;
  bool required = false;
  std::vector<std::string> allowed_values;  // 若非空，则必须为其中之一
  std::optional<double> min_value;          // 数值下限（适用于 Int/Double）
  std::optional<double> max_value;          // 数值上限（适用于 Int/Double）
};

struct PayloadValidationResult {
  bool valid = true;
  std::vector<std::string> errors;
  std::string message;
};

class PayloadSchema {
 public:
  PayloadSchema& AddField(const std::string& key, const FieldConstraint& constraint);

  PayloadValidationResult Validate(const OrderPayload& payload) const;

  // 内置预制 schema
  static PayloadSchema CreateTransportSchema();
  static PayloadSchema CreateStationTransportSchema();
  static PayloadSchema CreateStationOrderBatchSchema();

 private:
  std::unordered_map<std::string, FieldConstraint> fields_;
};

}  // namespace amr_dispatcher_core::workflow
