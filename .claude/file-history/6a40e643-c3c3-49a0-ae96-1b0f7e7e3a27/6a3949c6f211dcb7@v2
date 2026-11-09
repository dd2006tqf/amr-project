#include "amr_dispatcher_tools/alert_webhook_client.hpp"

#include <gtest/gtest.h>

#include <mutex>
#include <string>
#include <thread>

namespace amr_dispatcher_tools {
namespace {

// ---------------------------------------------------------------- URL 解析

TEST(WebhookUrlTest, ParsesHostPortAndPath) {
  std::string error;
  const auto url = ParseHttpUrl("http://alerts.example.com:9000/hooks/amr", &error);
  ASSERT_TRUE(url.has_value()) << error;
  EXPECT_EQ(url->host, "alerts.example.com");
  EXPECT_EQ(url->port, 9000);
  EXPECT_EQ(url->path, "/hooks/amr");
}

TEST(WebhookUrlTest, DefaultsPortTo80AndPathToRoot) {
  std::string error;
  const auto url = ParseHttpUrl("http://127.0.0.1", &error);
  ASSERT_TRUE(url.has_value()) << error;
  EXPECT_EQ(url->port, 80);
  EXPECT_EQ(url->path, "/");
}

// 企业微信/钉钉把 token 放在 query 上，拒绝 query 会让真实 webhook 直接不可用。
TEST(WebhookUrlTest, KeepsQueryStringInPath) {
  std::string error;
  const auto url = ParseHttpUrl("http://hook.local/send?key=abc123&x=1", &error);
  ASSERT_TRUE(url.has_value()) << error;
  EXPECT_EQ(url->path, "/send?key=abc123&x=1");
}

TEST(WebhookUrlTest, BareQueryGetsRootPath) {
  std::string error;
  const auto url = ParseHttpUrl("http://hook.local?key=abc", &error);
  ASSERT_TRUE(url.has_value()) << error;
  EXPECT_EQ(url->path, "/?key=abc");
}

// https 必须**显式拒绝**而不是静默明文发送：告警里可能带站点名与故障细节。
TEST(WebhookUrlTest, HttpsIsRejectedWithClearReason) {
  std::string error;
  EXPECT_FALSE(ParseHttpUrl("https://alerts.example.com/hook", &error).has_value());
  EXPECT_TRUE(error.find("https") != std::string::npos) << error;
}

TEST(WebhookUrlTest, RejectsMalformedUrls) {
  std::string error;
  EXPECT_FALSE(ParseHttpUrl("", &error).has_value());
  EXPECT_FALSE(ParseHttpUrl("alerts.example.com/hook", &error).has_value());
  EXPECT_FALSE(ParseHttpUrl("http://", &error).has_value());
  EXPECT_FALSE(ParseHttpUrl("http://host:abc/hook", &error).has_value());
  EXPECT_FALSE(ParseHttpUrl("http://host:0/hook", &error).has_value());
  EXPECT_FALSE(ParseHttpUrl("http://host:70000/hook", &error).has_value());
  // userinfo 会被当成 host 的一部分发出去（凭据泄露面），直接拒绝
  EXPECT_FALSE(ParseHttpUrl("http://user:pass@host/hook", &error).has_value());
  EXPECT_TRUE(error.find("userinfo") != std::string::npos) << error;
}

// ---------------------------------------------------------------- 报文组装

TEST(WebhookRequestTest, BuildsPostWithContentLengthAndClose) {
  ParsedUrl url;
  url.host = "127.0.0.1";
  url.port = 18111;
  url.path = "/alert";
  const std::string body = "{\"schema\":\"amr.alert.v1\"}";
  const std::string request = BuildWebhookPost(url, body);

  EXPECT_EQ(request.rfind("POST /alert HTTP/1.1\r\n", 0), 0u);
  EXPECT_TRUE(request.find("Host: 127.0.0.1:18111\r\n") != std::string::npos);
  EXPECT_TRUE(request.find("Content-Type: application/json\r\n") != std::string::npos);
  EXPECT_TRUE(request.find("Content-Length: " + std::to_string(body.size()) + "\r\n") !=
              std::string::npos);
  // 不带 Connection: close 时部分服务端会保持连接不 flush，发送线程只能靠超时脱身
  EXPECT_TRUE(request.find("Connection: close\r\n") != std::string::npos);
  // 头部与体之间必须恰好一个空行，且体原样附加
  const auto sep = request.find("\r\n\r\n");
  ASSERT_NE(sep, std::string::npos);
  EXPECT_EQ(request.substr(sep + 4), body);
}

TEST(WebhookRequestTest, OmitsDefaultPortInHostHeader) {
  ParsedUrl url;
  url.host = "hook.local";
  url.port = 80;
  url.path = "/x";
  const std::string request = BuildWebhookPost(url, "{}");
  EXPECT_TRUE(request.find("Host: hook.local\r\n") != std::string::npos);
  EXPECT_TRUE(request.find("hook.local:80") == std::string::npos);
}

// ---------------------------------------------------------------- 端到端（真 socket）

// 起一个只接受一次连接、回固定响应的本地服务端。
class LoopbackServer {
 public:
  explicit LoopbackServer(const std::string& response) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;  // 让内核挑端口
    ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    ::listen(fd_, 1);
    thread_ = std::thread([this, response]() {
      const int client = ::accept(fd_, nullptr, nullptr);
      if (client < 0) return;
      char buffer[2048];
      std::string received;
      // 读到请求头结束即可（body 大小已知，这里不解析，读一次通常够）
      const ssize_t n = ::recv(client, buffer, sizeof(buffer), 0);
      if (n > 0) received.assign(buffer, static_cast<std::size_t>(n));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        request_ = received;
      }
      ::send(client, response.data(), response.size(), MSG_NOSIGNAL);
      ::close(client);
    });
  }

  ~LoopbackServer() {
    if (fd_ >= 0) ::close(fd_);
    if (thread_.joinable()) thread_.join();
  }

  int port() const { return port_; }
  std::string request() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return request_;
  }

 private:
  int fd_ = -1;
  int port_ = 0;
  std::thread thread_;
  mutable std::mutex mutex_;
  std::string request_;
};

TEST(WebhookPostTest, PostsAndAccepts2xx) {
  LoopbackServer server("HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nok");
  ParsedUrl url;
  url.host = "127.0.0.1";
  url.port = server.port();
  url.path = "/alert";
  const auto outcome = PostWebhook(url, "{\"a\":1}", 2000);
  EXPECT_EQ(outcome.result, WebhookResult::kOk);
  EXPECT_EQ(outcome.status_code, 200);
  const std::string received = server.request();
  EXPECT_TRUE(received.rfind("POST /alert HTTP/1.1", 0) == 0u);
  EXPECT_TRUE(received.find("{\"a\":1}") != std::string::npos);
}

// 5xx 必须是失败：webhook 接收端挂了，我们的告警其实没送到。
TEST(WebhookPostTest, Non2xxIsFailureWithStatusCode) {
  LoopbackServer server("HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n\r\n");
  ParsedUrl url;
  url.host = "127.0.0.1";
  url.port = server.port();
  const auto outcome = PostWebhook(url, "{}", 2000);
  EXPECT_EQ(outcome.result, WebhookResult::kBadStatus);
  EXPECT_EQ(outcome.status_code, 503);
}

TEST(WebhookPostTest, UnreachablePortFailsFast) {
  ParsedUrl url;
  url.host = "127.0.0.1";
  url.port = 1;  // 几乎不可能有服务监听
  const auto outcome = PostWebhook(url, "{}", 500);
  EXPECT_EQ(outcome.result, WebhookResult::kConnectFailed);
}

TEST(WebhookPostTest, ResultNamesAreStableForMetrics) {
  // 指标标签取值必须稳定：这些字符串会被 amr_alert_webhook_total{result="..."} 使用
  EXPECT_STREQ(WebhookResultName(WebhookResult::kOk), "ok");
  EXPECT_STREQ(WebhookResultName(WebhookResult::kConnectFailed), "connect_failed");
  EXPECT_STREQ(WebhookResultName(WebhookResult::kTimeout), "timeout");
  EXPECT_STREQ(WebhookResultName(WebhookResult::kBadStatus), "bad_status");
}

}  // namespace
}  // namespace amr_dispatcher_tools
