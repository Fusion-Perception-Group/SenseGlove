#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>

//Constants
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#define E 2.718281828459045235360287471352662497757247093699959574966967627724076630353

//Macros
#define REP0(X)
#define REP1(X) X
#define REP2(X) REP1(X) X
#define REP3(X) REP2(X) X
#define REP4(X) REP3(X) X
#define REP5(X) REP4(X) X
#define REP6(X) REP5(X) X
#define REP7(X) REP6(X) X
#define REP8(X) REP7(X) X
#define REP9(X) REP8(X) X
#define REP10(X) REP9(X) X

#define REP(HUNDREDS,TENS,ONES,X) \
  REP##HUNDREDS(REP10(REP10(X))) \
  REP##TENS(REP10(X)) \
  REP##ONES(X)

#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define ISSIGNED(X) ((X)-1<(X)0)
#define ISFLOAT(X) (((float)((X)3.0/2)==(float)1.5)*sizeof(X)/sizeof(float))
#define BITPRINT(X) birPrint(&##X,sizeof(##X),stdout);
#define SWAP(a, b) \
  { \
    typeof (a) temp = a; \
    a = b; \
    b = temp; \
  }

//Typedefs
typedef uint32_t sort_t;

//Structures

//Unions
typedef union num_ptrs_t_proto
{
    uint8_t *u8;
    uint16_t *u16;
    uint32_t *u32;
    uint64_t *u64;
    int8_t *i8;
    int16_t *i16;
    int32_t *i32;
    int64_t *i64;
    float *f;
    double *d;
} num_ptrs_t;

//Enumerations
typedef enum month_t_proto
{
  JAN=1,FEB,MAR,APR,MAY,JUN,JUL,AUG,SEP,OCT,NOV,DEC
} month_t;
typedef enum num_type_t_proto
{
  INT8,INT16,INT32,INT64,INT128,UINT8,UINT16,UINT32,UINT64,UINT128,FLOAT,DOUBLE
} num_type_t;

//Functions
const char *getSysArch();
const char *getSysOS();
char *base64(const void *item,const size_t size);
sort_t *hybridSort(sort_t *base, size_t nmemb,  const unsigned MAX_RAM_MB);
sort_t *quickSort(sort_t *base, size_t nmemb);
sort_t *r_quickSort(sort_t *base, const size_t nmemb);//recursive implementation
sort_t *radixSort(sort_t *base, const size_t nmemb);
sort_t *radixSort_32(sort_t *base, const size_t nmemb);
void *partial_rsortX(void *base, const size_t nmemb,void *aux, const size_t size, const num_type_t type);//type: 0->unsigned 1->signed 2->float

//inline funtions
static inline bool getSysEndian()//returns 1 for little endian, 0 for big endian
{
  uint32_t x=1;
  return *(uint8_t*)&x;
}

static inline bool isExpOf2(const long unsigned n)
{
    return !(n & (n - 1));
}

static inline uint8_t reverseByte(uint8_t byte) {
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4; // Swap nibbles
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2; // Swap pairs
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1; // Swap individual bits
    return byte;
}

static inline void *swap(void *a,void *b,const size_t size)
{
    uint8_t temp[size];
    memcpy(temp,a,size);
    memcpy(a,b,size);
    memcpy(b,temp,size);
    return a;
}

static inline void *swap_64(void *a,void *b)
{
    uint64_t temp = *(uint64_t *)a;
    *(uint64_t *)a = *(uint64_t *)b;
    *(uint64_t *)b = temp;
    return a;
}

static inline void *swap_32(void *a,void *b)
{
    uint32_t temp = *(uint32_t *)a;
    *(uint32_t *)a = *(uint32_t *)b;
    *(uint32_t *)b = temp;
    return a;
}

static inline void *swap_16(void *a,void *b)
{
    uint16_t temp = *(uint16_t *)a;
    *(uint16_t *)a = *(uint16_t *)b;
    *(uint16_t *)b = temp;
    return a;
}

static inline void *swap_8(void *a,void *b)
{
    uint8_t temp = *(uint8_t *)a;
    *(uint8_t *)a = *(uint8_t *)b;
    *(uint8_t *)b = temp;
    return a;
}

static inline void *mapToRange(void *array, const size_t size, const num_type_t type, long unsigned min, long unsigned max)
{
    if(type<UINT8)
        swap_64(&min,&max);
    long unsigned range=max-min;
    switch (type)
    {
        case INT8:
            for(size_t i = 0; i < size; ++i)
                ((uint8_t *)array)[i] = (((uint8_t *)array)[i] % range) + min;
            break;
        case INT16:
            for(size_t i = 0; i < size; ++i)
                ((uint16_t *)array)[i] = (((uint16_t *)array)[i] % range) + min;
            break;
        case INT32:
            for(size_t i = 0; i < size; ++i)
                ((uint32_t *)array)[i] = (((uint32_t *)array)[i] % range) + min;
            break;
        case INT64:
            for(size_t i = 0; i < size; ++i)
                ((uint64_t *)array)[i] = (((uint64_t *)array)[i] % range) + min;
            break;
        case UINT8:
            for(size_t i = 0; i < size; ++i)
                ((uint8_t *)array)[i] = (((uint8_t *)array)[i] % range) + min;
            break;
        case UINT16:
            for(size_t i = 0; i < size; ++i)
                ((uint16_t *)array)[i] = (((uint16_t *)array)[i] % range) + min;
            break;
        case UINT32:
            for(size_t i = 0; i < size; ++i)
                ((uint32_t *)array)[i] = (((uint32_t *)array)[i] % range) + min;
            break;
        case UINT64:
            for(size_t i = 0; i < size; ++i)
                ((uint64_t *)array)[i] = (((uint64_t *)array)[i] % range) + min;
            break;
        default:
            return NULL;
    }
    return array;
}

static inline sort_t *insertSort(sort_t *base, const size_t nmemb)
{
    for(sort_t *p = base + 1, tmp; p < base + nmemb; ++p)
        for(sort_t *q=p;q > base && *(q - 1) > *q;--q)
        {
            tmp=*q;
            *q=*(q-1);
            *(q-1)=tmp;
        }
    return base;
}

#ifdef __cplusplus
}
#endif
