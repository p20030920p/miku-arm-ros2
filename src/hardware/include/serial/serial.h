// ============================================================================
// serial/serial.h —— POSIX 版串口库（接口兼容 wjwwood/serial 的常用子集）
//
// 背景：原工程是 ROS 1 的 catkin 包，用 <serial/serial.h>（wjwwood/serial）。
//       ROS 2 Jazzy 不再提供 ros-jazzy-serial，因此这里实现一个
//       API 兼容的替代品，直接基于 POSIX termios，无需额外依赖。
//
// 已实现（工程里实际用到的全部接口）：
//   serial::Serial        : setPort / setBaudrate / setTimeout /
//                           open / close / isOpen / available / read / write
//   serial::Timeout       : simpleTimeout()
//   serial::SerialException、serial::IOException
//                          （IOException 继承 SerialException，保持原库的
//                            异常层次，这样 catch 顺序不变）
//
// 语义对齐原库：
//   - read() 超时返回已读到的字节（可能是 0），不抛异常
//   - read()/write() 在设备消失（USB 拔出、单片机复位）时抛 IOException，
//     调用方据此 close() 并重连 —— 与原工程的重连逻辑一致
// ============================================================================
#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace serial {

// ---------------------------------------------------------------------------
// 异常类型
// ---------------------------------------------------------------------------
class SerialException : public std::runtime_error {
public:
  explicit SerialException(const std::string & what)
  : std::runtime_error(what) {}
};

class IOException : public SerialException {
public:
  explicit IOException(const std::string & what)
  : SerialException(what) {}
};

// ---------------------------------------------------------------------------
// 超时配置（对齐原库字段，便于直接照搬原代码）
// ---------------------------------------------------------------------------
class Timeout {
public:
  // 毫秒；read_timeout_constant = 0 且 multiplier = 0 表示阻塞式（不超时）
  uint32_t inter_byte_timeout = 0;
  uint32_t read_timeout_constant = 0;
  uint32_t read_timeout_multiplier = 0;
  uint32_t write_timeout_constant = 0;
  uint32_t write_timeout_multiplier = 0;

  Timeout() = default;

  Timeout(uint32_t inter_byte, uint32_t read_const, uint32_t read_mult,
          uint32_t write_const, uint32_t write_mult)
  : inter_byte_timeout(inter_byte),
    read_timeout_constant(read_const),
    read_timeout_multiplier(read_mult),
    write_timeout_constant(write_const),
    write_timeout_multiplier(write_mult)
  {}

  // 原库最常用的工厂函数：读超时 = timeout 毫秒
  static Timeout simpleTimeout(uint32_t timeout)
  {
    return Timeout(0, timeout, 0, timeout, 0);
  }
};

// ---------------------------------------------------------------------------
// 串口类
// ---------------------------------------------------------------------------
class Serial {
public:
  Serial() = default;
  ~Serial() { closeSilently(); }

  Serial(const Serial &) = delete;
  Serial & operator=(const Serial &) = delete;

  // ---------- 配置 ----------
  void setPort(const std::string & port)
  {
    port_ = port;
  }

  std::string getPort() const { return port_; }

  void setBaudrate(uint32_t baudrate)
  {
    baudrate_ = baudrate;
  }

  uint32_t getBaudrate() const { return baudrate_; }

  void setTimeout(const Timeout & timeout) { timeout_ = timeout; }

  const Timeout & getTimeout() const { return timeout_; }

  // ---------- 打开 / 关闭 ----------
  void open()
  {
    if (isOpen()) {
      return;
    }
    if (port_.empty()) {
      throw SerialException("Serial::open: 端口未设置（setPort 未调用）");
    }

    // O_NOCTTY：不把串口当作控制终端；O_NONBLOCK：先非阻塞打开，避免
    // 某些 USB 串口在 DCD 未就绪时卡住，随后再切回阻塞模式。
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      const std::string err = std::strerror(errno);
      closeSilently();
      throw IOException("Serial::open: 无法打开 " + port_ + "：" + err);
    }

    try {
      configure();
    } catch (...) {
      closeSilently();
      throw;
    }
  }

  void close()
  {
    if (fd_ >= 0) {
      const int fd = fd_;
      fd_ = -1;
      if (::close(fd) != 0) {
        throw IOException(std::string("Serial::close: ") + std::strerror(errno));
      }
    }
  }

  bool isOpen() const { return fd_ >= 0; }

  // ---------- 读写 ----------
  // 缓冲区里立即可读的字节数（无数据返回 0，不抛异常）
  size_t available()
  {
    if (fd_ < 0) {
      return 0;
    }
    int bytes = 0;
    if (::ioctl(fd_, FIONREAD, &bytes) != 0) {
      return 0;
    }
    return bytes > 0 ? static_cast<size_t>(bytes) : 0;
  }

  // 读最多 size 字节；读超时返回已读字节数（可能为 0）
  size_t read(uint8_t * buffer, size_t size)
  {
    if (fd_ < 0) {
      throw IOException("Serial::read: 端口未打开");
    }
    if (buffer == nullptr || size == 0) {
      return 0;
    }

    // 先把缓冲区里已有的数据取走，之后最多再等 read_timeout
    const int wait_ms = static_cast<int>(timeout_.read_timeout_constant);

    size_t total = 0;
    while (total < size) {
      const ssize_t n = ::read(fd_, buffer + total, size - total);
      if (n > 0) {
        total += static_cast<size_t>(n);
        // 有数据就继续把可读的一次取完，不额外等待
        if (available() == 0) {
          break;
        }
        continue;
      }
      if (n == 0) {
        break;   // 对端关闭
      }
      // n < 0
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (total > 0 || wait_ms <= 0) {
          break;   // 已经读到东西 / 不等待 → 直接返回
        }
        if (!waitReadable(wait_ms)) {
          break;   // 超时，返回已读到的（可能 0）
        }
        continue;
      }
      if (errno == EIO || errno == ENXIO || errno == ENODEV || errno == EBADF) {
        // 设备消失（拔 USB / 单片机复位）→ 交给上层重连
        throw IOException(std::string("Serial::read: 设备异常：") + std::strerror(errno));
      }
      throw IOException(std::string("Serial::read: ") + std::strerror(errno));
    }
    return total;
  }

  std::vector<uint8_t> read(size_t size)
  {
    std::vector<uint8_t> out(size);
    const size_t n = read(out.data(), size);
    out.resize(n);
    return out;
  }

  std::string readline(size_t /*size*/, const std::string & /*eol*/ = "\n")
  {
    throw SerialException("Serial::readline 未实现（本工程用不到）");
  }

  // 写全部字节；失败抛 IOException
  size_t write(const uint8_t * data, size_t size)
  {
    if (fd_ < 0) {
      throw IOException("Serial::write: 端口未打开");
    }
    if (data == nullptr || size == 0) {
      return 0;
    }

    size_t total = 0;
    while (total < size) {
      const ssize_t n = ::write(fd_, data + total, size - total);
      if (n >= 0) {
        total += static_cast<size_t>(n);
        continue;
      }
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (!waitWritable(static_cast<int>(timeout_.write_timeout_constant))) {
          throw IOException("Serial::write: 超时");
        }
        continue;
      }
      if (errno == EIO || errno == ENXIO || errno == ENODEV || errno == EBADF) {
        throw IOException(std::string("Serial::write: 设备异常：") + std::strerror(errno));
      }
      throw IOException(std::string("Serial::write: ") + std::strerror(errno));
    }
    return total;
  }

  // 兼容原库的模板写入（buffer / std::vector<uint8_t> / std::string）
  size_t write(const std::vector<uint8_t> & data)
  {
    return write(data.data(), data.size());
  }

  size_t write(const std::string & data)
  {
    return write(reinterpret_cast<const uint8_t *>(data.data()), data.size());
  }

  template <typename T, size_t N>
  size_t write(const T (&data)[N])
  {
    return write(reinterpret_cast<const uint8_t *>(data), N);
  }

  void flush()
  {
    if (fd_ >= 0) {
      ::tcflush(fd_, TCIOFLUSH);
    }
  }

private:
  // ---------- termios 配置 ----------
  void configure()
  {
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (::tcgetattr(fd_, &tty) != 0) {
      throw IOException(std::string("Serial::configure: tcgetattr 失败：") +
                        std::strerror(errno));
    }

    cfmakeraw(&tty);            // 8N1、无回显、无流控（等价原库默认配置）

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;    // 关闭硬件流控
    tty.c_cflag &= ~CSTOPB;     // 1 位停止位
    tty.c_cflag &= ~PARENB;     // 无校验
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8 位数据

    const speed_t speed = baudToSpeed(baudrate_);
    if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
      throw IOException("Serial::configure: 不支持的波特率 " +
                        std::to_string(baudrate_));
    }

    // 读超时：VTIME 以 0.1s 为单位；不足 100ms 按 1 计，避免变成完全阻塞
    int vtime = 0;
    if (timeout_.read_timeout_constant > 0) {
      vtime = static_cast<int>((timeout_.read_timeout_constant + 99) / 100);
    }
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = static_cast<cc_t>(vtime);

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw IOException(std::string("Serial::configure: tcsetattr 失败：") +
                        std::strerror(errno));
    }

    // 配置完成 → 切回阻塞模式，让 read() 的 VTIME 超时生效
    const int flags = ::fcntl(fd_, F_GETFL);
    if (flags >= 0) {
      ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }

    ::tcflush(fd_, TCIOFLUSH);   // 丢掉打开前的残留数据
  }

  static speed_t baudToSpeed(uint32_t baud)
  {
    switch (baud) {
      case 1200:    return B1200;
      case 2400:    return B2400;
      case 4800:    return B4800;
      case 9600:    return B9600;
      case 19200:   return B19200;
      case 38400:   return B38400;
      case 57600:   return B57600;
      case 115200:  return B115200;   // 本工程使用
      case 230400:  return B230400;
      case 460800:  return B460800;
      case 500000:  return B500000;
      case 921600:  return B921600;
      case 1000000: return B1000000;
      default:      return B0;
    }
  }

  bool waitReadable(int timeout_ms) const
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int r = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    return r > 0;
  }

  bool waitWritable(int timeout_ms) const
  {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd_, &wfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int r = ::select(fd_ + 1, nullptr, &wfds, nullptr, &tv);
    return r > 0;
  }

  void closeSilently()
  {
    if (fd_ >= 0) {
      const int fd = fd_;
      fd_ = -1;
      ::close(fd);
    }
  }

  std::string port_;
  uint32_t baudrate_ = 115200;
  Timeout timeout_ = Timeout::simpleTimeout(1000);
  int fd_ = -1;
};

}  // namespace serial
