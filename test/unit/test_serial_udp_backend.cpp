#include "amr_dispatcher_core/chassis/serial_backend.hpp"
#include "amr_dispatcher_core/chassis/udp_backend.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <vector>

using namespace amr_dispatcher_core::chassis;
using namespace std::chrono_literals;

namespace {

// PTY 对：master 由测试读取，slave 路径交给 SerialBackend 打开
struct PtyPair {
  int master = -1;
  std::string slave_path;

  ~PtyPair() {
    if (master >= 0) {
      close(master);
    }
  }
};

PtyPair OpenPty() {
  PtyPair pty;
  pty.master = posix_openpt(O_RDWR | O_NOCTTY);
  if (pty.master < 0) {
    return pty;
  }
  if (grantpt(pty.master) != 0 || unlockpt(pty.master) != 0) {
    close(pty.master);
    pty.master = -1;
    return pty;
  }
  const char* slave = ptsname(pty.master);
  if (slave == nullptr) {
    close(pty.master);
    pty.master = -1;
    return pty;
  }
  pty.slave_path = slave;
  // master 置非阻塞，便于排空式读取
  const int flags = fcntl(pty.master, F_GETFL, 0);
  fcntl(pty.master, F_SETFL, flags | O_NONBLOCK);
  return pty;
}

// 从 master 读到换行符（带超时），返回已读全部内容；超时返回超时前已读内容
std::string ReadUntilNewline(const int fd, const std::chrono::milliseconds timeout) {
  std::string acc;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  char buf[512];
  while (acc.find('\n') == std::string::npos) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }
    pollfd pfd{fd, POLLIN, 0};
    const auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const int ready = poll(&pfd, 1, static_cast<int>(remain.count()) + 1);
    if (ready <= 0) {
      break;
    }
    const ssize_t n = read(fd, buf, sizeof(buf));
    if (n > 0) {
      acc.append(buf, static_cast<std::size_t>(n));
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    } else {
      break;
    }
  }
  return acc;
}

// 排空 fd 中已缓冲的全部数据
void DrainFd(const int fd) {
  char buf[4096];
  while (read(fd, buf, sizeof(buf)) > 0) {
  }
}

// 回环 UDP 接收端（带 1s 读超时）
int OpenLoopbackReceiver(sockaddr_in* out_addr) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return -1;
  }
  timeval tv{.tv_sec = 1, .tv_usec = 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return -1;
  }
  socklen_t len = sizeof(addr);
  if (getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
    close(fd);
    return -1;
  }
  *out_addr = addr;
  return fd;
}

}  // namespace

// A5(a)：文本命令经 PTY 完整到达对端（EncodeCommand 全帧，无截断）
TEST(SerialBackendTest, WriteCommandDeliversFullLineToMaster) {
  PtyPair pty = OpenPty();
  ASSERT_GE(pty.master, 0);

  ChassisBackendConfig cfg;
  cfg.serial_device = pty.slave_path;
  cfg.serial_baud = 115200;
  cfg.protocol = "text";
  SerialBackend backend(cfg);
  std::string error;
  ASSERT_TRUE(backend.Open(&error)) << error;

  ChassisCommand cmd{.linear_x_mps = 1.5, .linear_y_mps = -0.5, .angular_z_radps = 0.25};
  ASSERT_TRUE(backend.WriteCommand(cmd, &error)) << error;

  const std::string received = ReadUntilNewline(pty.master, 500ms);
  EXPECT_EQ(received, "CMD 1.5000 -0.5000 0.2500\n");
}

// A5(b)：从端缓冲写满（对端不读）时非阻塞 write 返回 EAGAIN —— 必须返回 false 计入丢包，
// 而不是旧语义的"假装成功"
TEST(SerialBackendTest, WouldBlockReturnsFalse) {
  PtyPair pty = OpenPty();
  ASSERT_GE(pty.master, 0);

  ChassisBackendConfig cfg;
  cfg.serial_device = pty.slave_path;
  cfg.serial_baud = 115200;
  cfg.protocol = "text";
  SerialBackend backend(cfg);
  std::string error;
  ASSERT_TRUE(backend.Open(&error)) << error;

  ChassisCommand cmd{.linear_x_mps = 1.5, .linear_y_mps = 0.0, .angular_z_radps = 0.25};
  // 对端不读：写满 PTY 内核缓冲（实测约 18KB，每帧 25 字节，取上限 2000 帧 ~50KB）后必然 would-block
  bool saw_would_block = false;
  for (int i = 0; i < 2000 && !saw_would_block; ++i) {
    if (!backend.WriteCommand(cmd, &error)) {
      saw_would_block = true;
    }
  }
  EXPECT_TRUE(saw_would_block);
  EXPECT_FALSE(error.empty());

  // 对端排空后应恢复正常写入（部分写循环 + 重试路径不被 EAGAIN 卡死）
  DrainFd(pty.master);
  error.clear();
  EXPECT_TRUE(backend.WriteCommand(cmd, &error)) << error;
  EXPECT_EQ(ReadUntilNewline(pty.master, 500ms), "CMD 1.5000 0.0000 0.2500\n");
}

// A5(c)：未打开直接写返回 false
TEST(SerialBackendTest, WriteWhenNotOpenReturnsFalse) {
  ChassisBackendConfig cfg;
  cfg.serial_device = "/dev/null-not-a-serial-port";
  SerialBackend backend(cfg);
  std::string error;
  EXPECT_FALSE(backend.WriteCommand(ChassisCommand{}, &error));
  EXPECT_FALSE(error.empty());
}

// A5(d)：UDP 数据报完整到达接收端（单报文原子语义，无拼接/截断）
TEST(UdpBackendTest, WriteCommandDeliversFullDatagram) {
  sockaddr_in recv_addr{};
  const int recv_fd = OpenLoopbackReceiver(&recv_addr);
  ASSERT_GE(recv_fd, 0);

  ChassisBackendConfig cfg;
  cfg.udp_host = "127.0.0.1";
  cfg.udp_port = ntohs(recv_addr.sin_port);
  cfg.protocol = "text";
  UdpBackend backend(cfg);
  std::string error;
  ASSERT_TRUE(backend.Open(&error)) << error;

  ChassisCommand cmd{.linear_x_mps = 1.5, .linear_y_mps = -0.5, .angular_z_radps = 0.25};
  ASSERT_TRUE(backend.WriteCommand(cmd, &error)) << error;

  char buf[512];
  const ssize_t n = recv(recv_fd, buf, sizeof(buf), 0);
  ASSERT_GT(n, 0);
  const std::string received(buf, static_cast<std::size_t>(n));
  EXPECT_EQ(received, "CMD 1.5000 -0.5000 0.2500\n");
  close(recv_fd);
}

// A5(e)：UDP 未打开直接写返回 false；无效主机 Open 失败
TEST(UdpBackendTest, WriteWhenNotOpenReturnsFalse) {
  ChassisBackendConfig cfg;
  cfg.udp_host = "127.0.0.1";
  cfg.udp_port = 65500;
  UdpBackend backend(cfg);
  std::string error;
  EXPECT_FALSE(backend.WriteCommand(ChassisCommand{}, &error));
  EXPECT_FALSE(error.empty());

  ChassisBackendConfig bad_cfg;
  bad_cfg.udp_host = "not-an-ip-address";
  UdpBackend bad_backend(bad_cfg);
  EXPECT_FALSE(bad_backend.Open(&error));
  EXPECT_FALSE(error.empty());
}
