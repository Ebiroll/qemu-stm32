/* Native implementation of soft float functions. Only a single status
   context is supported */
#include "softfloat.h"
#include <math.h>

void set_float_rounding_mode(int val STATUS_PARAM)
{
    STATUS(float_rounding_mode) = val;
#if defined(_BSD) && !defined(__APPLE__)
    fpsetround(val);
#elif defined(__arm__)
    /* nothing to do */
#else
    fesetround(val);
#endif
}

#ifdef FLOATX80
void set_floatx80_rounding_precision(int val STATUS_PARAM)
{
    STATUS(floatx80_rounding_precision) = val;
}
#endif

#if defined(_BSD)
#define lrint(d)		((int32_t)rint(d))
#define llrint(d)		((int64_t)rint(d))
#endif

#if defined(__powerpc__)

/* correct (but slow) PowerPC rint() (glibc version is incorrect) */
double qemu_rint(double x)
{
    double y = 4503599627370496.0;
    if (fabs(x) >= y)
        return x;
    if (x < 0) 
        y = -y;
    y = (x + y) - y;
    if (y == 0.0)
        y = copysign(y, x);
    return y;
}

#define rint qemu_rint
#endif

/*----------------------------------------------------------------------------
| Software IEC/IEEE integer-to-floating-point conversion routines.
*----------------------------------------------------------------------------*/
float32 int32_to_float32(int v STATUS_PARAM)
{
    return (float32)v;
}

float64 int32_to_float64(int v STATUS_PARAM)
{
    return (float64)v;
}

#ifdef FLOATX80
floatx80 int32_to_floatx80(int v STATUS_PARAM)
{
    return (floatx80)v;
}
#endif
float32 int64_to_float32( int64_t v STATUS_PARAM)
{
    return (float32)v;
}
float64 int64_to_float64( int64_t v STATUS_PARAM)
{
    return (float64)v;
}
#ifdef FLOATX80
floatx80 int64_to_floatx80( int64_t v STATUS_PARAM)
{
    return (floatx80)v;
}
#endif

/*----------------------------------------------------------------------------
| Software IEC/IEEE single-precision conversion routines.
*----------------------------------------------------------------------------*/
int float32_to_int32( float32 a STATUS_PARAM)
{
    return lrintf(a);
}
int float32_to_int32_round_to_zero( float32 a STATUS_PARAM)
{
    return (int)a;
}
int64_t float32_to_int64( float32 a STATUS_PARAM)
{
    return llrintf(a);
}

int64_t float32_to_int64_round_to_zero( float32 a STATUS_PARAM)
{
    return (int64_t)a;
}

float64 float32_to_float64( float32 a STATUS_PARAM)
{
    return a;
}
#ifdef FLOATX80
floatx80 float32_to_floatx80( float32 a STATUS_PARAM)
{
    return a;
}
#endif

/*----------------------------------------------------------------------------
| Software IEC/IEEE single-precision operations.
*----------------------------------------------------------------------------*/
float32 float32_round_to_int( float32 a STATUS_PARAM)
{
    return rintf(a);
}

float32 float32_rem( float32 a, float32 b STATUS_PARAM)
{
    return remainderf(a, b);
}

float32 float32_sqrt( float32 a STATUS_PARAM)
{
    return sqrtf(a);
}
char float32_compare( float32 a, float32 b STATUS_PARAM )
{
    if (a < b) {
        return -1;
    } else if (a == b) {
        return 0;
    } else if (a > b) {
        return 1;
    } else {
        return 2;
    }
}
char float32_compare_quiet( float32 a, float32 b STATUS_PARAM )
{
    if (isless(a, b)) {
        return -1;
    } else if (a == b) {
        return 0;
    } else if (isgreater(a, b)) {
        return 1;
    } else {
        return 2;
    }
}
char float32_is_signaling_nan( float32 a1)
{
    float32u u;
    uint32_t a;
    u.f = a1;
    a = u.i;
    return ( ( ( a>>22 ) & 0x1FF ) == 0x1FE ) && ( a & 0x003FFFFF );
}

/*----------------------------------------------------------------------------
| Software IEC/IEEE double-precision conversion routines.
*----------------------------------------------------------------------------*/
int float64_to_int32( float64 a STATUS_PARAM)
{
    return lrint(a);
}
int float64_to_int32_round_to_zero( float64 a STATUS_PARAM)
{
    return (int)a;
}
int64_t float64_to_int64( float64 a STATUS_PARAM)
{
    return llrint(a);
}
int64_t float64_to_int64_round_to_zero( float64 a STATUS_PARAM)
{
    return (int64_t)a;
}
float32 float64_to_float32( float64 a STATUS_PARAM)
{
    return a;
}
#ifdef FLOATX80
floatx80 float64_to_floatx80( float64 a STATUS_PARAM)
{
    return a;
}
#endif
#ifdef FLOAT128
float128 float64_to_float128( float64 a STATUS_PARAM)
{
    return a;
}
#endif

/*----------------------------------------------------------------------------
| Software IEC/IEEE double-precision operations.
*----------------------------------------------------------------------------*/
float64 float64_round_to_int( float64 a STATUS_PARAM )
{
#if defined(__arm__)
    switch(STATUS(float_rounding_mode)) {
    default:
    case float_round_nearest_even:
        asm("rndd %0, %1" : "=f" (a) : "f"(a));
        break;
    case float_round_down:
        asm("rnddm %0, %1" : "=f" (a) : "f"(a));
        break;
    case float_round_up:
        asm("rnddp %0, %1" : "=f" (a) : "f"(a));
        break;
    case float_round_to_zero:
        asm("rnddz %0, %1" : "=f" (a) : "f"(a));
        break;
    }
#else
    return rint(a);
#endif
}

float64 float64_rem( float64 a, float64 b STATUS_PARAM)
{
    return remainder(a, b);
}

float64 float64_sqrt( float64 a STATUS_PARAM)
{
    return sqrt(a);
}
char float64_compare( float64 a, float64 b STATUS_PARAM )
{
    if (a < b) {
        return -1;
    } else if (a == b) {
        return 0;
    } else if (a > b) {
        return 1;
    } else {
        return 2;
    }
}
char float64_compare_quiet( float64 a, float64 b STATUS_PARAM )
{
    if (isless(a, b)) {
        return -1;
    } else if (a == b) {
        return 0;
    } else if (isgreater(a, b)) {
        return 1;
    } else {
        return 2;
    }
}
char float64_is_signaling_nan( float64 a1)
{
    float64u u;
    uint64_t a;
    u.f = a1;
    a = u.i;
    return
           ( ( ( a>>51 ) & 0xFFF ) == 0xFFE )
        && ( a & LIT64( 0x0007FFFFFFFFFFFF ) );

}

#ifdef FLOATX80

/*----------------------------------------------------------------------------
| Software IEC/IEEE extended double-precision conversion routines.
*----------------------------------------------------------------------------*/
int floatx80_to_int32( floatx80 a STATUS_PARAM)
{
    return lrintl(a);
}
int floatx80_to_int32_round_to_zero( floatx80 a STATUS_PARAM)
{
    return (int)a;
}
int64_t floatx80_to_int64( floatx80 a STATUS_PARAM)
{
    return llrintl(a);
}
int64_t floatx80_to_int64_round_to_zero( floatx80 a STATUS_PARAM)
{
    return (int64_t)a;
}
float32 floatx80_to_float32( floatx80 a STATUS_PARAM)
{
    return a;
}
float64 floatx80_to_float64( floatx80 a STATUS_PARAM)
{
    return a;
}

/*----------------------------------------------------------------------------
| Software IEC/IEEE extended double-precision operations.
*----------------------------------------------------------------------------*/
floatx80 floatx80_round_to_int( floatx80 a STATUS_PARAM)
{
    return rintl(a);
}
floatx80 floatx80_rem( floatx80 a, floatx80 b STATUS_PARAM)
{
    return remainderl(a, b);
}
floatx80 floatx80_sqrt( floatx80 a STATUS_PARAM)
{
    return sqrtl(a);
}
char floatx80_compare( floatx80 a, floatx80 b STATUS_PARAM )
{
    if (a < b) {
        return -1;
    } else if (a == b) {
        return 0;
    } else if (a > b) {
        return 1;
    } else {
        return 2;
    }
}
char floatx80_compare_quiet( floatx80 a, floatx80 b STATUS_PARAM )
{
    if (isless(a, b)) {
        return -1;
    } else if (a == b) {
        return 0;
    } else if (isgreater(a, b)) {
        return 1;
    } else {
        return 2;
    }
}
char floatx80_is_signaling_nan( floatx80 a1)
{
    floatx80u u;
    u.f = a1;
    return ( ( u.i.high & 0x7FFF ) == 0x7FFF ) && (bits64) ( u.i.low<<1 );
}

#endif
