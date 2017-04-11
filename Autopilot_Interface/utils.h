#pragma once
#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdint.h>

void usleep(__int64 usec);
uint64_t get_time_usec();

#endif