#include "amr_dispatcher_core/dispatcher/audit_log.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;
using namespace amr_dispatcher_core::dispatcher;

namespace {

fs::path TempPath(const std::string& name) {
  static std::uintptr_t counter = 0;
  return fs::temp_directory_path() /
         ("amr_audit_log_" + std::to_string(::getpid()) + "_" + std::to_string(++counter) + "_" +
          name);
}

AuditRecord MakeRecord() {
  AuditRecord record;
  record.stamp_iso8601 = "2026-10-23T10:00:00Z";
  record.operator_id = "operator_zhang";
  record.action = "force_clear";
  record.vehicle_id = "amr_01";
  record.mission_id = "m_7";
  record.reason = "vehicle scrapped";
  record.released_resources = {"route_node:station_pickup_A", "route_edge:B__A"};
  return record;
}

}  // namespace

TEST(AuditLogTest, SerializedRecordHasAllFields) {
  const std::string line = SerializeAuditRecordJsonl(MakeRecord());
  EXPECT_NE(line.find("\"stamp\":\"2026-10-23T10:00:00Z\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"operator_id\":\"operator_zhang\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"action\":\"force_clear\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"vehicle_id\":\"amr_01\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"mission_id\":\"m_7\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"reason\":\"vehicle scrapped\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"released_resources\":[\"route_node:station_pickup_A\","),
            std::string::npos) << line;
}

TEST(AuditLogTest, SerializedRecordIsSingleLine) {
  AuditRecord record = MakeRecord();
  record.reason = "line one\nline two\r\nwith \"quotes\" and \\backslash";
  const std::string line = SerializeAuditRecordJsonl(record);
  EXPECT_EQ(line.find('\n'), std::string::npos) << line;
  EXPECT_EQ(line.find('\r'), std::string::npos) << line;
  // 转义后的换行仍在内容里，但以字面量 \n 出现
  EXPECT_NE(line.find("line one\\nline two"), std::string::npos) << line;
  EXPECT_NE(line.find("\\\"quotes\\\""), std::string::npos) << line;
}

TEST(AuditLogTest, AppendWritesOneLinePerRecord) {
  const fs::path path = TempPath("audit.jsonl");
  std::string message;
  ASSERT_TRUE(AppendAuditRecord(path, MakeRecord(), &message)) << message;
  AuditRecord second = MakeRecord();
  second.action = "restore";
  second.vehicle_id = "amr_02";
  ASSERT_TRUE(AppendAuditRecord(path, second, &message)) << message;

  std::ifstream in(path);
  std::stringstream buffer;
  buffer << in.rdbuf();
  const std::string content = buffer.str();
  std::size_t lines = 0;
  for (char c : content) {
    if (c == '\n') ++lines;
  }
  EXPECT_EQ(lines, 2u) << content;
  // 追加不覆盖：两条记录都在
  EXPECT_NE(content.find("force_clear"), std::string::npos);
  EXPECT_NE(content.find("restore"), std::string::npos);
  fs::remove(path);
}

TEST(AuditLogTest, AppendCreatesParentDirectories) {
  const fs::path dir = TempPath("audit_dir");
  const fs::path path = dir / "logs" / "audit.jsonl";
  std::string message;
  ASSERT_TRUE(AppendAuditRecord(path, MakeRecord(), &message)) << message;
  EXPECT_TRUE(fs::exists(path));
  fs::remove_all(dir);
}

TEST(AuditLogTest, EmptyPathFailsWithoutThrowing) {
  std::string message;
  EXPECT_FALSE(AppendAuditRecord(fs::path{}, MakeRecord(), &message));
  EXPECT_FALSE(message.empty());
}

TEST(AuditLogTest, EmptyReleasedResourcesRendersEmptyArray) {
  AuditRecord record = MakeRecord();
  record.released_resources.clear();
  const std::string line = SerializeAuditRecordJsonl(record);
  EXPECT_NE(line.find("\"released_resources\":[]"), std::string::npos) << line;
}
