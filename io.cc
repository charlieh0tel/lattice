#include "io.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sysexits.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>

extern std::string argv0;

void Fail(int exit_code, const std::string &message) {
  std::cerr << argv0 << ": " << message << std::endl;
  std::exit(exit_code);
}

void ReadOrLose(int fd, void *buf, ssize_t count) {
  ssize_t rc = read(fd, buf, count);
  if (rc != count) {
    std::ostringstream out;
    out << "failed to read bytes, rc=" << rc << ", errno=" << errno << " ("
        << std::strerror(errno) << ")";
    Fail(EX_IOERR, out.str());
  }
}

void WriteOrLose(int fd, const void *buf, ssize_t count) {
  ssize_t rc = write(fd, buf, count);
  if (rc != count) {
    std::ostringstream out;
    out << "failed to write bytes, rc=" << rc << ", errno=" << errno << " ("
        << std::strerror(errno) << ")";
    Fail(EX_IOERR, out.str());
  }
}

std::vector<std::uint8_t> ReadFileOrLose(const std::string &path) {
  const int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    Fail(EX_NOINPUT, "failed to open " + path + " for reading");
  }
  struct stat stat;
  const int rc = fstat(fd, &stat);
  if (rc < 0) {
    Fail(EX_IOERR, "failed to stat " + path);
  }

  std::vector<std::uint8_t> bytes(stat.st_size);
  ReadOrLose(fd, &bytes[0], bytes.size());

  if (close(fd) < 0) {
    Fail(EX_IOERR, "failed to close " + path);
  }

  return bytes;
}

void WriteFileOrLose(const std::string &path, const std::string &contents) {
  const int fd = open(path.c_str(), O_WRONLY);
  if (fd < 0) {
    Fail(EX_NOINPUT, "failed to open " + path + " for writing");
  }
  WriteOrLose(fd, contents.c_str(), contents.size());
  if (close(fd) < 0) {
    Fail(EX_IOERR, "failed to close " + path);
  }
}

bool FileExists(const std::string &path) {
  const int rc = access(path.c_str(), F_OK);
  return rc == 0;
}
