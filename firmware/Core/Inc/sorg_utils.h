/*************************************************************
 * @file sorg_utils.h
 * @author MicroOrange
 * @brief 
 * @date 2022/12/27
 *************************************************************/

#ifndef F4_25R3916_SORG_UTILS_H
#define F4_25R3916_SORG_UTILS_H

#include "string.h"
#include "stdint.h"
#include "crash.h"
/**
 * must include other utils before this file
 */


#ifndef MAX
#define MAX(a, b)               \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a > _b ? _a : _b;      \
    })
#endif

#ifndef MIN
#define MIN(a, b)               \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a < _b ? _a : _b;      \
    })
#endif

#ifndef ROUND_UP_TO
#define ROUND_UP_TO(a, b)       \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a / _b + !!(_a % _b);  \
    })
#endif

#ifndef CLAMP
#define CLAMP(x, upper, lower) (MIN(upper, MAX(x, lower)))
#endif

#ifndef COUNT_OF
#define COUNT_OF(x) (sizeof(x) / sizeof(x[0]))
#endif

#ifndef SWAP
#define SWAP(x, y)     \
    do {                    \
        typeof(x) SWAP = x; \
        x = y;              \
        y = SWAP;           \
    } while(0)
#endif

#ifndef PLACE_IN_SECTION
#define PLACE_IN_SECTION(x) __attribute__((section(x)))
#endif

#ifndef ALIGN
#define ALIGN(n) __attribute__((aligned(n)))
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif

#ifndef UNUSED
#define UNUSED(X) (void)(X)
#endif

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif

#ifndef TOSTRING
#define TOSTRING(x) STRINGIFY(x)
#endif

#ifndef REVERSE_BYTES_U32
#define REVERSE_BYTES_U32(x)                                                        \
    ((((x)&0x000000FF) << 24) | (((x)&0x0000FF00) << 8) | (((x)&0x00FF0000) >> 8) | \
     (((x)&0xFF000000) >> 24))
#endif

#ifndef BIT
#define BIT(x, n) (((x) >> (n)) & 1)
#endif

#ifndef BIT_SET
#define BIT_SET(x, n)      \
    ({                          \
        __typeof__(x) _x = (1); \
        (x) |= (_x << (n));     \
    })
#endif

#ifndef BIT_CLEAR
#define BIT_CLEAR(x, n) ((x) &= ~(1 << (n)))
#endif

void tick_us_init();
void tick_us_callback();
uint64_t tick_get_us();

#endif //F4_25R3916_SORG_UTILS_H
