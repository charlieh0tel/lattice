#include "io.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sysexits.h>
#include <unistd.h>

#include <boost/format.hpp>
#include <iostream>

extern std::string argv0;

void Fail(int exit_code, const std::string &message) {
  std::cerr << argv0 << ": " << message << std::endl;
  std::exit(exit_code);
}

void ReadOrLose(int fd, void *buf, ssize_t count) {
  ssize_t rc = read(fd, buf, count);
  if (rc != count) {
    Fail(EX_IOERR,
         (boost::format("failed to read bytes, rc=%1%, errno=%2%") % rc % errno)
             .str());
  }
}

void WriteOrLose(int fd, const void *buf, ssize_t count) {
  ssize_t rc = write(fd, buf, count);
  if (rc != count) {
    Fail(EX_IOERR, (boost::format("failed to write bytes, rc=%1%, errno=%2%") %
                    rc % errno)
                       .str());
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
