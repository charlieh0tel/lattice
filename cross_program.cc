#include <fcntl.h>
#include <linux/i2c-dev.h>
#ifdef NEED_I2C_H
#include <linux/i2c.h>
#endif
#include <sys/ioctl.h>
#include <sysexits.h>
#include <unistd.h>
#ifdef WIRING_PI
#include <wiringPi.h>
#endif
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "io.h"

#ifdef NEED_I2C_H
typedef __u8 i2c_buf_t;
typedef __u16 i2c_len_t;
#else
typedef char i2c_buf_t;
typedef short i2c_len_t;
#endif

// This limit is baked into the kernel, but (sigh) is not exposed in
// any header.
const int kLinuxMaxI2CMessageSize = 8192;

// Lattice activation key to enable programming mode.
constexpr uint8_t kCrosslinkActivationKey[] = {0xa4, 0xc6, 0xf4, 0x8a};

// Lattice opcodes.
constexpr uint8_t kOpcodeIDCODE[] = {0xe0, 0x00, 0x00, 0x00};
constexpr uint8_t kOpcodeENABLE[] = {0xc6, 0x00, 0x00, 0x00};
constexpr uint8_t kOpcodeERASE[] = {0x0e, 0x00, 0x00, 0x00};
constexpr uint8_t kOpcodeREAD_STATUS[] = {0x3c, 0x00, 0x00, 0x00};
constexpr uint8_t kOpcodeLSC_INIT_ADDRESS[] = {0x46, 0x00, 0x00, 0x00};
constexpr uint8_t kOpcodeLSC_BITSTREAM_BURST[] = {0x7a, 0x00, 0x00, 0x00};
constexpr uint8_t kOpcodeREAD_USER_CODE[] = {0xc0, 0x00, 0x00, 0x00};
constexpr uint8_t kOpcodeDISABLE[] = {0x26, 0x00, 0x00, 0x00};

// Required delays between operations.
constexpr auto kCRESETBLowDelay = std::chrono::milliseconds(
    1000);  // Lattice now says 0 (was 1000 ms in previous doc).
constexpr auto kCRESETBHighDelay = std::chrono::milliseconds(10);
constexpr auto kEnableDelay = std::chrono::milliseconds(1);
constexpr auto kEraseDelay = std::chrono::milliseconds(
    100);  // Lattice says 0 (was 5000 ms in previous doc)

// Flags in READ_STATUS value.
constexpr uint32_t kREAD_STATUSDoneFlag = (1 << 8);
constexpr uint32_t kREAD_STATUSBusyFlag = (1 << 12);
constexpr uint32_t kREAD_STATUSFailFlag = (1 << 13);

#define NELEMENTS(A) (sizeof(A) / sizeof(A[0]))

std::string argv0;

void Usage() {
  std::cerr << "usage: " << argv0
            << " i2c-device i2c-address gpio_number binary" << std::endl;
  std::exit(EX_USAGE);
}

std::string DumpBuffer(const void *buf, const int count, const int max_count) {
  std::ostringstream out;
  const uint8_t *u8_buf = static_cast<const uint8_t *>(buf);
  const int brief_count = std::min(count, max_count);
  out << "[";
  for (int i = 0; i < brief_count; ++i) {
    if (i != 0) {
      out << " ";
    }
    out << std::hex << std::setw(2) << static_cast<int>(u8_buf[i]);
  }
  if (count > brief_count) {
    out << " ... ";
  }
  out << "]";
  return out.str();
}

template <typename T>
void CrosslinkComamndOrLose(const int fd, const T &command) {
#ifdef NOISY
  std::cout << "W: " << DumpBuffer(command, sizeof(command), 32) << std::endl;
#endif
  WriteOrLose(fd, command, sizeof(command));
}

template <typename T>
uint32_t CrosslinkReadOrLose(const int fd, const int i2c_address,
                             const T &command) {
  uint8_t reply[4];
  struct i2c_msg msgs[] = {
      {
          .addr = static_cast<__u16>(i2c_address),
          .flags = 0,
          .len = sizeof(command),
          .buf = reinterpret_cast<i2c_buf_t *>(
              const_cast<unsigned char *>(command)),
      },
      {
          .addr = static_cast<__u16>(i2c_address),
          .flags = I2C_M_RD,
          .len = sizeof(reply),
          .buf = reinterpret_cast<i2c_buf_t *>(reply),
      },
  };
  struct i2c_rdwr_ioctl_data ioctl_data = {.msgs = msgs, .nmsgs = 2};
#ifdef NOISY
  std::cout << "W: " << DumpBuffer(msgs[0].buf, msgs[0].len, 32) << std::endl;
#endif
  int rc = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (rc < 0) {
    std::ostringstream out;
    out << "failed with error on ioctl I2C_RDWR, rc=" << rc
        << ", errno=" << errno << " (" << std::strerror(errno) << ")";
    Fail(EX_IOERR, out.str());
  }
  if (rc != static_cast<int>(ioctl_data.nmsgs)) {
    std::ostringstream out;
    out << "failed with transfer all messages on I2C_RDWR, rc=" << rc;
    Fail(EX_IOERR, out.str());
  }
#ifdef NOISY
  std::cout << "R: " << DumpBuffer(msgs[1].buf, msgs[1].len, 32) << std::endl;
#endif
  return ((reply[0] << 24) | (reply[1] << 16) | (reply[2] << 8) |
          (reply[3] << 0));
}

void CrosslinkSendBitstreamOrLose(const int fd, const int i2c_address,
                                  const std::vector<std::uint8_t> &binary) {
  std::vector<i2c_msg> msgs;
  struct i2c_msg command_msg = {
      .addr = static_cast<__u16>(i2c_address),
      .flags = 0,
      .len = sizeof(kOpcodeLSC_BITSTREAM_BURST),
      .buf = reinterpret_cast<i2c_buf_t *>(
          const_cast<unsigned char *>(kOpcodeLSC_BITSTREAM_BURST)),
  };
  msgs.push_back(command_msg);

  for (size_t i = 0; i < binary.size(); i += kLinuxMaxI2CMessageSize) {
    int n_remaining = binary.size() - i;
    int n_write = std::min(kLinuxMaxI2CMessageSize, n_remaining);
    struct i2c_msg bitstream_msg = {
        .addr = static_cast<__u16>(i2c_address),
        .flags = I2C_M_NOSTART,
        .len = static_cast<i2c_len_t>(n_write),
        .buf = reinterpret_cast<i2c_buf_t *>(
            const_cast<unsigned char *>(&(binary[i]))),
    };
    msgs.push_back(bitstream_msg);
  }
  struct i2c_rdwr_ioctl_data ioctl_data = {
      .msgs = &(msgs[0]), .nmsgs = static_cast<__u32>(msgs.size())};

#ifdef NOISY
  for (size_t i = 0; i < msgs.size(); ++i) {
    std::cout << "#" << std::setw(2) << i << ", adddr=0x" << std::hex
              << std::setw(2) << msgs[i].addr << ", flags=0x" << std::hex
              << std::setw(4) << msgs[i].flags << ", len=0x" << std::hex
              << std::setw(4) << msgs[i].len << ", buf=0x" << std::hex
              << std::setw(16) << reinterpret_cast<ptrdiff_t>(&(msgs[i].buf[0]))
              << " " << DumpBuffer(msgs[i].buf, msgs[i].len, 16) << std::endl;
  }
#endif

  int rc = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (rc < 0) {
    std::ostringstream out;
    out << "failed with error on ioctl I2C_RDWR, rc=" << rc
        << ", errno=" << errno << " (" << std::strerror(errno) << ")";
    Fail(EX_IOERR, out.str());
  }
  if (rc != static_cast<int>(ioctl_data.nmsgs)) {
    std::ostringstream out;
    out << "failed with transfer all messages on I2C_RDWR, rc=" << rc;
    Fail(EX_IOERR, out.str());
  }
}

#ifndef WIRING_PI
std::string SysGPIOPath(int gpio_number, const std::string &what) {
  std::ostringstream path_out;
  path_out << "/sys/class/gpio/gpio" << gpio_number << "/" << what;
  return path_out.str();
}
#endif

void SetupGPIO(int cresetb_gpio_num) {
// Setup CRESETB GPIO.
#ifdef WIRING_PI
  if (wiringPiSetup() == -1) {
    Fail(EX_IOERR, "Failed to initialize wiringPi");
  }
  pinMode(cresetb_gpio_num, OUTPUT);
#else
  if (!FileExists(SysGPIOPath(cresetb_gpio_num, "value"))) {
    WriteFileOrLose("/sys/class/gpio/export", std::to_string(cresetb_gpio_num));
    WriteFileOrLose(SysGPIOPath(cresetb_gpio_num, "direction"), "out");
  }
#endif
}

void SetCRESETBOutput(const int gpio_number, int value) {
#ifdef NOISY
  std::cout << "CRESETB <- " << value << std::endl;
#endif
#ifdef WIRING_PI
  digitalWrite(gpio_number, value);
#else
  WriteFileOrLose(SysGPIOPath(gpio_number, "value"), std::to_string(value));
#endif
}

void CrosslinkProgram(const int cresetb_gpio_num, int i2c_fd, int i2c_address,
                      const std::vector<std::uint8_t> &binary) {
  // 1. Initialize.
  std::cout << "1. Initialize" << std::endl;
  {
    SetCRESETBOutput(cresetb_gpio_num, 0);
    std::this_thread::sleep_for(kCRESETBLowDelay);
    CrosslinkComamndOrLose(i2c_fd, kCrosslinkActivationKey);
    SetCRESETBOutput(cresetb_gpio_num, 1);
    std::this_thread::sleep_for(kCRESETBHighDelay);
  }

  // 2. Check IDCODE.
  std::cout << "2. Check IDCODE." << std::endl;
  {
    const uint32_t id_code =
        CrosslinkReadOrLose(i2c_fd, i2c_address, kOpcodeIDCODE);
    std::cout << "IDCODE = 0x" << std::hex << std::setw(8) << id_code
              << std::endl;
  }

  // 3. Enable SRAM Programming Mode
  std::cout << "3. Enable SRAM Programming Mode." << std::endl;
  {
    CrosslinkComamndOrLose(i2c_fd, kOpcodeENABLE);
    std::this_thread::sleep_for(kEnableDelay);
  }

  // 4. Erase SRAM
  std::cout << "4. Erase SRAM." << std::endl;
  {
    CrosslinkComamndOrLose(i2c_fd, kOpcodeERASE);
    // Lattice now says this delay is not required, but sometimes we
    // see BUSY in the status register without it.  Could poll the
    // status register for !BUSY ... or just delay.
    std::this_thread::sleep_for(kEraseDelay);
  }

  // 5. Read Status Register
  std::cout << "5. Read Status Register." << std::endl;
  {
    const uint32_t status =
        CrosslinkReadOrLose(i2c_fd, i2c_address, kOpcodeREAD_STATUS);
    const bool busy = status & kREAD_STATUSBusyFlag;
    const bool fail = status & kREAD_STATUSFailFlag;
    std::cout << "0x" << std::hex << std::setw(8) << status
              << " => {busy = " << (busy ? "yes" : "no")
              << ", fail = " << (fail ? "yes" : "no") << "}" << std::endl;
    if (fail) {
      Fail(EX_IOERR, "device signaled failure");
    }
    if (busy) {
      Fail(EX_IOERR, "device busy");
    }
  }

  // 6. Program SRAM.
  std::cout << "6. Program SRAM." << std::endl;
  {
    CrosslinkComamndOrLose(i2c_fd, kOpcodeLSC_INIT_ADDRESS);
    CrosslinkSendBitstreamOrLose(i2c_fd, i2c_address, binary);
  }

  // 7. Verify USERCODE.
  std::cout << "7. Verify USERCODE." << std::endl;
  {
    const uint32_t user_code =
        CrosslinkReadOrLose(i2c_fd, i2c_address, kOpcodeREAD_USER_CODE);
    std::cout << std::hex << std::setw(8) << user_code << std::endl;
  }

  // 8. Read Status Register.
  std::cout << "5. Read Status Register." << std::endl;
  {
    const uint32_t status =
        CrosslinkReadOrLose(i2c_fd, i2c_address, kOpcodeREAD_STATUS);
    const bool busy = status & kREAD_STATUSBusyFlag;
    const bool fail = status & kREAD_STATUSFailFlag;
    const bool done = status & kREAD_STATUSDoneFlag;
    std::cout << std::hex << std::setw(8) << status
              << " => {busy = " << (busy ? "yes" : "no")
              << ", fail = " << (fail ? "yes" : "no")
              << ", done = " << (done ? "yes" : "no") << "}" << std::endl;
    if (fail) {
      Fail(EX_IOERR, "device signaled failure");
    }
    if (busy) {
      Fail(EX_IOERR, "device is busy");
    }
    if (!done) {
      Fail(EX_IOERR, "device is not done (programming failed)");
    }
  }

  // 9. Exit Programming Mode.
  std::cout << "9. Exit Programming Mode." << std::endl;
  { CrosslinkComamndOrLose(i2c_fd, kOpcodeDISABLE); }
}

int main(int argc, char **argv) {
  argv0 = argv[0];

  if (argc != 5) {
    Usage();
  }

  const std::string i2c_path(argv[1]);
  const int i2c_address = std::stoi(argv[2], 0, 0);
  const int cresetb_gpio_num = std::stoi(argv[3]);
  const std::string binary_path(argv[4]);

  if (i2c_address <= 0 || i2c_address > 0x7f) {
    std::ostringstream out;
    out << "bad i2c address: " << i2c_address;
    Fail(EX_DATAERR, out.str());
  }

  const auto &binary = ReadFileOrLose(binary_path);

  SetupGPIO(cresetb_gpio_num);

  int i2c_fd = open(i2c_path.c_str(), O_RDWR);
  if (i2c_fd < 0) {
    Fail(EX_NOINPUT, "failed top open i2c_device " + i2c_path);
  }

  if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  if (ioctl(i2c_fd, I2C_RETRIES, 5) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  // A big timeout is required or the bitstream burst will timeout.
  if (ioctl(i2c_fd, I2C_TIMEOUT, 60 * 100 /* tens of ms */) < 0) {
    Fail(EX_IOERR, "failed to set I2C timeout");
  }

  CrosslinkProgram(cresetb_gpio_num, i2c_fd, i2c_address, binary);

  if (close(i2c_fd) < 0) {
    Fail(EX_IOERR, "failed to close I2C device");
  }
}
