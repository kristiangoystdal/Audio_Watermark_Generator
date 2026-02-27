#pragma once

#ifndef ENABLE_LOG
#define ENABLE_LOG 0
#endif

#if ENABLE_LOG
#include <stdio.h>
#define LOGF(...) printf(__VA_ARGS__)
#else
#define LOGF(...)                                                              \
  do {                                                                         \
  } while (0)
#endif