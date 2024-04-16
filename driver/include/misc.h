/* MIT License

Copyright (c) 2022 Po Jui Shih
Copyright (c) 2022 Hassaan Saadat
Copyright (c) 2022 Sri Parameswaran
Copyright (c) 2022 Hasindu Gamaarachchi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef HARU_MISC_H
#define HARU_MISC_H

#include <stdio.h>
#include <stdint.h>

// TODO: add defines of error codes

/*
 * Register getter and setter
 */
#define _reg_set(BaseAddress, RegOffset, Data) \
    *(volatile uint32_t*)((BaseAddress) + (RegOffset >> 2)) = (uint32_t)(Data)
#define _reg_get(BaseAddress, RegOffset) \
    *(volatile uint32_t*)((BaseAddress) + (RegOffset >> 2))

#define HARU_INFO(msg) \
    fprintf(stderr, "INFO: %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, "%s", msg);

#define HARU_LOG_PREFIX "[HARU_LOG] %s: " /* TODO function before debug */
#define HARU_ERROR_PREFIX "[%s::HARU_ERROR]\033[1;31m "
#define HARU_STATUS_PREFIX "[HARU_STATUS]\033[1;36m "
#define HARU_LOG(msg, ...) { \
    fprintf(stderr, HARU_LOG_PREFIX msg \
            " At %s:%d\n", \
            __func__, __VA_ARGS__, __FILE__, __LINE__ - 1); \
}
#define HARU_ERROR(msg, ...) { \
    fprintf(stderr, HARU_ERROR_PREFIX msg \
            " At %s:%d\n", \
            __func__, __VA_ARGS__, __FILE__, __LINE__ - 1); \
}
// uint32_t haru_errno = 0;

#define HARU_STATUS(msg, ...) { \
    fprintf(stderr, HARU_STATUS_PREFIX msg "\n" \
             ,__VA_ARGS__); \
}
#define HARU_MALLOC_CHK(ret) { \
    if ((ret) == NULL) { \
        HARU_ERROR("%s", "Malloc Failed") \
    } \
}

#endif // MISC_H