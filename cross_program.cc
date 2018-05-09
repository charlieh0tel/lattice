#include <cstddef>
#include <cstdlib>
#include <cinttypes>
#include <iostream>
#include <string>
#include <vector>
#include <boost/format.hpp>
#include <sysexits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <wiringPi.h>

constexpr uint32_t kCResetBHighDelayMicroseconds = 10 * 1000;
constexpr uint32_t kEnableSRAMProgramDelayMicroseconds = 1 * 1000;
constexpr uint32_t kEraseSRAMDelayMicroseconds = 1000 * 1000;
constexpr uint32_t kProgramSRAMDelayMicroseconds = 10 * 1000;

constexpr uint8_t kCrosslinkActivationKey[] = { 0xa4, 0xc6, 0xf4, 0x8a };
constexpr uint8_t kOpcodeIDCODE[] = { 0xe0, 0x00, 0x00, 0x00 };
constexpr uint8_t kOpcodeENABLE_SRAM_PROGRAM[] = { 0xc6, 0x00, 0x00, 0x00 };
constexpr uint8_t kOpcodeERASE_SRAM[] = { 0x0e, 0x00, 0x00, 0x00 };
constexpr uint8_t kOpcodeREAD_STATUS[] = { 0x3c, 0x00, 0x00, 0x00 };
constexpr uint8_t kOpcodeLSC_INIT_ADDRESS[] = { 0x46, 0x00, 0x00, 0x00 };
constexpr uint8_t kOpcodeLSC_BITSTREAM_BURST[] = { 0x7a, 0x00, 0x00, 0x00 };
constexpr uint8_t kOpcodeLSC_READ_USER_CODE[] = { 0xc0, 0x00, 0x00, 0x00 };
constexpr uint8_t kOpcodeDISABLE[] = { 0x26, 0x00, 0x00, 0x00 };

constexpr uint32_t kStatusDoneFlag = (1<<8);
constexpr uint32_t kStatusBusyFlag = (1<<12);
constexpr uint32_t kStatusFailFlag = (1<<13);

#define NELEMENTS(A) (sizeof(A)/sizeof(A[0]))

std::string argv0;

void Usage() {
  std::cerr << (boost::format("usage: %1% i2c-device i2c-address gpio_number binary")
		% argv0)
	    << std::endl;
  std::exit(EX_USAGE);
}

void Fail(int exit_code, const std::string &message) {
  std::cerr << argv0 << ": " << message << std::endl;
  std::exit(exit_code);
}

void ReadOrLose(int fd, void *buf, ssize_t count) {
  ssize_t rc = read(fd, buf, count);
  if (rc != count) {
    Fail(EX_IOERR, (boost::format("failed to read bytes, rc=%1%, errno=%2%") 
		    % rc % errno).str());
  }
}

void WriteOrLose(int fd, const void *buf, ssize_t count) {
  ssize_t rc = write(fd, buf, count);
  if (rc != count) {
    Fail(EX_IOERR, (boost::format("failed to write bytes, rc=%1%, errno=%2%")
		    % rc % errno).str());
  }
}

std::vector<std::uint8_t> ReadFileOrLose(const std::string &path) {
  const int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    Fail(EX_NOINPUT, (boost::format("failed to open %1%") % path).str());
  }
  struct stat stat;
  const int rc = fstat(fd, &stat);
  if (rc < 0) {
    Fail(EX_IOERR, (boost::format("failed to stat %1%") % path).str());
  }

  std::vector<std::uint8_t> bytes(stat.st_size);
  ReadOrLose(fd, &bytes[0], bytes.size());

  if (close(fd) < 0) {
    Fail(EX_IOERR, (boost::format("failed to close %1%") % path).str());
  }

  return bytes;
}

void WriteI2COrLose(int fd, const void *buf, ssize_t count) {
#ifdef NOISY
  std::cout << "W:";
  constexpr int max_noise = 32;
  int brief_count = std::min(count, max_noise);
  for (int i = 0; i < brief_count; ++i) {
    std::cout << boost::format(" %02X") % static_cast<int>(static_cast<const uint8_t *>(buf)[i]);
  }
  if (count > brief_count) {
    std::cout << " ...";
  }
  std::cout << std::endl;
#endif
  WriteOrLose(fd, buf, count);
}

void ReadI2COrLose(int fd, void *buf, ssize_t count) {
  ReadOrLose(fd, buf, count);
}

uint32_t ReadI2CBigEndian32BitsOrLose(int fd) {
  uint8_t bytes[4];
  ReadI2COrLose(fd, &bytes, sizeof(bytes));
  return ((bytes[0] << 24) |
	  (bytes[1] << 16) |
	  (bytes[2] << 8) |
	  (bytes[3] << 0));
}

template <typename T>
void CrossComamndOrLose(int fd, const T &command) {
  WriteI2COrLose(fd, command, sizeof(command));
}

template <typename T>
uint32_t CrossReadOrLose(int fd, int addr, const T &command) {
  uint8_t reply[4];
  struct i2c_msg msgs[] = {
    {
      .addr = static_cast<__u16>(addr),
      .flags = 0,
      .len = sizeof(command),
      .buf = reinterpret_cast<char *>(const_cast<unsigned char *>(command)),
    },
    {
      .addr = static_cast<__u16>(addr),
      .flags = I2C_M_RD,
      .len = sizeof(reply),
      .buf = reinterpret_cast<char *>(reply),
    },
  };
  struct i2c_rdwr_ioctl_data ioctl_data = {
    .msgs = msgs,
    .nmsgs = 2
  };
  int rc = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (rc < 0) {
    Fail(EX_IOERR, (boost::format("failed to I2C_RDWR, rc=%1%, errno=%2%")
		    % rc % errno).str());
  }
  return ((reply[0] << 24) |
	  (reply[1] << 16) |
	  (reply[2] << 8) |
	  (reply[3] << 0));
}

void CrossWriteBitstreamOrLose(int fd, int addr, const uint8_t *command, int size) {
  uint8_t reply[4];
  struct i2c_msg msgs[] = {
    {
      .addr = static_cast<__u16>(addr),
      .flags = 0,
      .len = static_cast<short>(size),
      .buf = reinterpret_cast<char *>(const_cast<uint8_t *>(command)),
    },
    {
      .addr = static_cast<__u16>(addr),
      .flags = I2C_M_RD,
      .len = sizeof(reply),
      .buf = reinterpret_cast<char *>(reply),
    },
  };
  struct i2c_rdwr_ioctl_data ioctl_data = {
    .msgs = msgs,
    .nmsgs = 2
  };
  int rc = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (rc < 0) {
    Fail(EX_IOERR, (boost::format("failed to I2C_RDWR, rc=%1%, errno=%2%")
		    % rc % errno).str());
  }
}


void CrossProgram(const int gpio_number, int i2c_fd, int i2c_address,
		  const std::vector<std::uint8_t> &binary) {
  // 1. Initialize.
  {
    std::cout << "1. Initialize" << std::endl;
    std::cout << "CRESETB <- 0" << std::endl;
    digitalWrite(gpio_number, 0);
    CrossComamndOrLose(i2c_fd, kCrosslinkActivationKey);
    digitalWrite(gpio_number, 1);
    std::cout << "CRESETB <- 1" << std::endl;
    usleep(kCResetBHighDelayMicroseconds);
  }

  // 2. Check IDCODE.
  {
    std::cout << "2. Check IDCODE." << std::endl;
    const uint32_t id_code = CrossReadOrLose(i2c_fd, i2c_address, kOpcodeIDCODE);
    std::cout << (boost::format("idcode = 0x%08X") % id_code) << std::endl;
  }

  // 3. Enable SRAM Programming Mode
  {
    std::cout << "3. Enable SRAM Programming Mode." << std::endl;
    CrossComamndOrLose(i2c_fd, kOpcodeENABLE_SRAM_PROGRAM);
    usleep(kEnableSRAMProgramDelayMicroseconds);
  }

  // 4. Erase SRAM
  {
    std::cout << "4. Erase SRAM." << std::endl;
    CrossComamndOrLose(i2c_fd, kOpcodeERASE_SRAM);
    usleep(kEraseSRAMDelayMicroseconds);
  }

  // 5. Read Status Register
  {
    std::cout << "5. Read Status Register." << std::endl;
    const uint32_t status = CrossReadOrLose(i2c_fd, i2c_address, kOpcodeREAD_STATUS);
    std::cout << (boost::format("status = 0x%08X") % status) << std::endl;
    const bool busy = status & kStatusBusyFlag;
    const bool fail = status & kStatusFailFlag;
    std::cout << "busy = " << (busy ? "yes" : "no") << std::endl;
    std::cout << "fail = " << (fail ? "yes" : "no") << std::endl;
    if (busy) {
      Fail(EX_IOERR, (boost::format("device busy, status=0x%08x") % status).str());
    }
    if (fail) {
      Fail(EX_IOERR, (boost::format("device fail, status=0x%08x") % status).str());
    }
  }

  // 6. Program SRAM.
  {
  std::cout << "6. Program SRAM." << std::endl;
  CrossComamndOrLose(i2c_fd, kOpcodeLSC_INIT_ADDRESS);
#if 0
  std::vector<std::uint8_t> sram_program(binary);
  sram_program.insert(sram_program.begin(),
    std::begin(kOpcodeLSC_BITSTREAM_BURST),
    std::end(kOpcodeLSC_BITSTREAM_BURST));

  // Without STOP ...  write bistream
  WriteI2COrLose(i2c_fd, &sram_program[0], sram_program.size());
#else
  int n_remaining = binary.size();
  auto it = binary.begin();
  const int chunk_size = 1024;
  while (n_remaining > 0) {
    std::vector<std::uint8_t> chunk;
    auto n_to_write = std::min(chunk_size, n_remaining);
    chunk.insert(chunk.begin(),
		 std::begin(kOpcodeLSC_BITSTREAM_BURST),
		 std::end(kOpcodeLSC_BITSTREAM_BURST));
    chunk.insert(chunk.end(), it, it + n_to_write);
    WriteI2COrLose(i2c_fd, &chunk[0], chunk.size());
    n_remaining -= n_to_write;
    it += n_to_write;
  }    
#endif
  usleep(kProgramSRAMDelayMicroseconds);

}

  // 7. Verify USERCODE.
  {
    std::cout << "7. Verify USERCODE." << std::endl;
    const uint32_t user_code = CrossReadOrLose(i2c_fd, i2c_address, kOpcodeLSC_READ_USER_CODE);
    std::cout << (boost::format("user_code = 0x%08X") % user_code) << std::endl;
  }
  
  // 8. Read Status Register.
  {
    std::cout << "5. Read Status Register." << std::endl;
    const uint32_t status = CrossReadOrLose(i2c_fd, i2c_address, kOpcodeREAD_STATUS);
    std::cout << (boost::format("status = 0x%08X") % status) << std::endl;
    const bool busy = status & kStatusBusyFlag;
    const bool fail = status & kStatusFailFlag;
    const bool done = status & kStatusDoneFlag;
    std::cout << "busy = " << (busy ? "yes" : "no") << std::endl;
    std::cout << "fail = " << (fail ? "yes" : "no") << std::endl; 
    std::cout << "done = " << (done ? "yes" : "no") << std::endl;
    if (busy) {
      Fail(EX_IOERR, (boost::format("device busy, status=0x%08x") % status).str());
    }
    if (fail) {
      Fail(EX_IOERR, (boost::format("device fail, status=0x%08x") % status).str());
    }
    if (!done) {
      Fail(EX_IOERR, (boost::format("device not done, status=0x%08x") % status).str());
    }
  }

  // 9. Exit Programming Mode.
  {
    std::cout << "9. Exit Programming Mode." << std::endl;
    CrossComamndOrLose(i2c_fd, kOpcodeDISABLE);
  }
}

int main(int argc, char **argv) {
  argv0 = argv[0];

  if (argc != 5) {
    Usage();
  }

  const std::string i2c_path(argv[1]);
  const int i2c_address = std::stoi(argv[2], 0, 0);
  const int gpio_number = std::stoi(argv[3]);
  const std::string binary_path(argv[4]);

  if (i2c_address <= 0 || i2c_address > 0x7f) {
    Fail(EX_DATAERR,
         (boost::format("bad i2c address: %1%") % i2c_address).str());
  }

  const auto &binary = ReadFileOrLose(binary_path);
#ifdef NOISY
  std::cout << "Binary is " << binary.size() << " bytes." << std::endl;
#endif

  if (wiringPiSetup() == -1) {
    Fail(EX_IOERR, "Failed to initialize wiringPi");
  }

  pinMode(gpio_number, OUTPUT);
  digitalWrite(gpio_number, 1);

  int i2c_fd = open(i2c_path.c_str(), O_RDWR);
  if (i2c_fd < 0) {
    Fail(EX_NOINPUT,
         (boost::format("failed to open i2c device %1%") % i2c_path).str());
  }

  if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  if (ioctl(i2c_fd, I2C_RETRIES, 5) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  if (ioctl(i2c_fd, I2C_TIMEOUT, 500 /* tens of ms */) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  CrossProgram(gpio_number, i2c_fd, i2c_address, binary);

  if (close(i2c_fd) < 0) {
    Fail(EX_IOERR, "failed to close I2C device");
  }

}  
