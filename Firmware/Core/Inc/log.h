#pragma once

#define ENABLE_LOG 1

#if ENABLE_LOG
#include <stdio.h>
#define LOGF(...) printf(__VA_ARGS__)
#else
#define LOGF(...)                                                              \
  do {                                                                         \
  } while (0)
#endif