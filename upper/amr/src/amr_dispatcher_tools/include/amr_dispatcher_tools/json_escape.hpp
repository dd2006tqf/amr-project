// JSON 字符串转义（纯函数）。
//
// 现状：网关所有响应字段都是裸拼（`"message":"` + res->message + `"`），
// 只要车辆 id / 拒绝原因里出现引号、反斜杠或换行，返回体就是非法 JSON，
// 前端 JSON.parse 直接抛异常。这里统一走一条转义路径。
#pragma once

#include <cstddef>
#include <cstdio>
#include <string>

namespace amr_dispatcher_tools {

// 按 RFC 8259 转义：引号、反斜杠、控制字符（< 0x20）转义，
// 其余字节原样保留（UTF-8 多字节序列不需要转义）。
inline std::string JsonEscape(const std::string& input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (const char raw : input) {
    const unsigned char c = static_cast<unsigned char>(raw);
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[7];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out.push_back(raw);
        }
        break;
    }
  }
  return out;
}

// 常用拼装：`"key":"<escaped>"`（含引号，不带逗号）。
inline std::string JsonStringPair(const std::string& key, const std::string& value) {
  return "\"" + JsonEscape(key) + "\":\"" + JsonEscape(value) + "\"";
}

}  // namespace amr_dispatcher_tools
