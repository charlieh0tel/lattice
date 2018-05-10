#ifndef _IO_H_
#define _IO_H_

#include <cinttypes>
#include <string>
#include <vector>

void Fail(int exit_code, const std::string &message);
void ReadOrLose(int fd, void *buf, ssize_t count);
void WriteOrLose(int fd, const void *buf, ssize_t count);
std::vector<std::uint8_t> ReadFileOrLose(const std::string &path);

#endif
