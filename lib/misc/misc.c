#include"./misc.h"

const char *getSysArch() 
{
    #if defined(__x86_64__) || defined(_M_X64)
    return "x86_64";
    #elif defined(i386) || defined(__i386__) || defined(__i386) || defined(_M_IX86)
    return "x86_32";
    #elif defined(__m68k__)
    return "M68K";
    #elif defined(__ARM_ARCH_2__)
    return "ARMv2";
    #elif defined(__ARM_ARCH_3__) || defined(__ARM_ARCH_3M__)
    return "ARMv3";
    #elif defined(__ARM_ARCH_4T__) || defined(__TARGET_ARM_4T)
    return "ARMv4T";
    #elif defined(__ARM_ARCH_5_) || defined(__ARM_ARCH_5E_)
    return "ARMv5"
    #elif defined(__ARM_ARCH_6T2_) || defined(__ARM_ARCH_6T2_)
    return "ARMv6T2";
    #elif defined(__ARM_ARCH_6__) || defined(__ARM_ARCH_6J__) || defined(__ARM_ARCH_6K__) || defined(__ARM_ARCH_6Z__) || defined(__ARM_ARCH_6ZK__)
    return "ARMv6";
    #elif defined(__ARM_ARCH_7__) || defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7S__)
    return "ARMv7";
    #elif defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7S__)
    return "ARMv7A";
    #elif defined(__ARM_ARCH_7R__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7S__)
    return "ARMv7R";
    #elif defined(__ARM_ARCH_7M__)
    return "ARMv7M";
    #elif defined(__ARM_ARCH_7S__)
    return "ARMv7S";
    #elif defined(__aarch64__) || defined(_M_ARM64)
    return "ARM64";
    #elif defined(mips) || defined(__mips__) || defined(__mips)
    return "MIPS";
    #elif defined(__sh__)
    return "SUPERH";
    #elif defined(__PPC64__) || defined(__ppc64__) || defined(_ARCH_PPC64)
    return "POWERPC64";
    #elif defined(__sparc__) || defined(__sparc)
    return "SPARC";
    #elif defined(__powerpc) || defined(__powerpc__) || defined(__powerpc64__) || defined(__POWERPC__) || defined(__ppc__) || defined(__PPC__) || defined(_ARCH_PPC)
    return "POWERPC";
    #else
    return NULL;
    #endif
}

const char *getSysOS()
{
    #if defined(__linux__)
    return "Linux";
    #elif defined(__APPLE__) || defined(__MACH__)
    return "macOS";
    #elif defined(_WIN32) || defined(_WIN64)
    return "Windows";
    #elif defined(__FreeBSD__)
    return "FreeBSD";
    #elif defined(__OpenBSD__)
    return "OpenBSD";
    #elif defined(__NetBSD__)
    #elif defined(__ANDROID__)
    return "Android";
    #elif defined(unix) || defined(__unix__) || defined(__unix)
    return "Unix";
    #else
    return "UNKNOWN";
    #endif
}

char *base64(const void *item,const size_t size)
{
    static const char *b64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t out_size = 4*((size+2)/3);
    char *out = (char*)malloc(out_size+1);
    return out;
}

sort_t *r_quickSort(sort_t *base, const size_t nmemb)
{
    #define q_swap(a,b) {sort_t tmp = a; a = b; b = tmp;}
    sort_t *pvt=base+nmemb/2,*l=base,*r=base+nmemb-1;
    if(nmemb<2)
        return base;
    if(*l>*pvt)         //inline 3-element insertion sort
        q_swap(*l,*pvt);
    if(*pvt>*r)
    {
        q_swap(*pvt,*r);
        if(*l>*pvt)
            q_swap(*l,*pvt);
    }
    while(l<r)
    {
        for(;*l<*pvt;++l);
        for(;*r>*pvt;--r);
        q_swap(*l,*r);
        if(l==pvt)
        {
            pvt=r;
            ++l;
        }
        else if(r==pvt)
        {
            pvt=l;
            --r;
        }
        else
            ++l,--r;
    }
    r_quickSort(base,pvt-base), r_quickSort(pvt+1,base+nmemb-pvt-1);
    return base;
    #undef q_swap
}

sort_t *quickSort(sort_t *base, size_t nmemb)
{
    #define q_swap(a,b) {sort_t tmp = a; a = b; b = tmp;}
    #define STACK_SIZE (CHAR_BIT*sizeof(size_t))
    if(nmemb<=1)
        return base;
    struct Layer
    {
        sort_t *base;
        size_t nmemb;
    } stack[STACK_SIZE], *layer;
    layer=stack;
    sort_t *pvt=base+nmemb/2, *l=base, *r=base+nmemb-1;
    do
    {
        if(*l>*pvt)         //inline 3-element insertion sort
            q_swap(*l,*pvt);
        if(*pvt>*r)
        {
            q_swap(*pvt,*r);
            if(*l>*pvt)
                q_swap(*l,*pvt);
        }
        while(l<r)
        {
            for(;*l<*pvt;++l);
            for(;*r>*pvt;--r);
            q_swap(*l,*r);
            if(l==pvt)
            {
                pvt=r;
                ++l;
            }
            else if(r==pvt)
            {
                pvt=l;
                --r;
            }
            else
                ++l,--r;
        }
        if(pvt==base)
            if(base+nmemb-1==pvt)
            {
                --layer;
                base=layer->base, nmemb=layer->nmemb;
            }
            else
            {
                nmemb=base+nmemb-1-pvt;
                base=pvt+1;
            }
        else if(base+nmemb-1==pvt)
            nmemb=pvt-base;
        else
        {
            layer->base=base, layer->nmemb=pvt-base;
            ++layer;
            nmemb=base+nmemb-1-pvt;
            base=pvt+1;
        }
        l=base, r=base+nmemb-1, pvt=base+nmemb/2;
    }while(layer>=stack);
    return base;
    #undef q_swap
    #undef STACK_SIZE
}

sort_t *radixSort(sort_t *base, const size_t nmemb)
{
    #define BIT 8
    #define UNIT_SIZE sizeof(sort_t)
    if(nmemb<2) return base;
    sort_t *aux = (sort_t*)malloc(nmemb*UNIT_SIZE);
    sort_t *src=(sort_t*)base,*dst=(sort_t*)aux;
    size_t cnt[UNIT_SIZE][1u<<BIT];
    const uint8_t sign_mask=(uint8_t)ISSIGNED(sort_t)<<7;
    if(!aux) return NULL;
    memset(cnt,0,sizeof(cnt));
    for(size_t i=nmemb;i--;)//Counting
    {
        switch(UNIT_SIZE)
        {
            case 16: ++cnt[UNIT_SIZE-16][(src[i]>>((UNIT_SIZE-16)*BIT))&0xff];
            case 15: ++cnt[UNIT_SIZE-15][(src[i]>>((UNIT_SIZE-15)*BIT))&0xff];
            case 14: ++cnt[UNIT_SIZE-14][(src[i]>>((UNIT_SIZE-14)*BIT))&0xff];
            case 13: ++cnt[UNIT_SIZE-13][(src[i]>>((UNIT_SIZE-13)*BIT))&0xff];
            case 12: ++cnt[UNIT_SIZE-12][(src[i]>>((UNIT_SIZE-12)*BIT))&0xff];
            case 11: ++cnt[UNIT_SIZE-11][(src[i]>>((UNIT_SIZE-11)*BIT))&0xff];
            case 10: ++cnt[UNIT_SIZE-10][(src[i]>>((UNIT_SIZE-10)*BIT))&0xff];
            case 9: ++cnt[UNIT_SIZE-9][(src[i]>>((UNIT_SIZE-9)*BIT))&0xff];
            case 8: ++cnt[UNIT_SIZE-8][(src[i]>>((UNIT_SIZE-8)*BIT))&0xff];
            case 7: ++cnt[UNIT_SIZE-7][(src[i]>>((UNIT_SIZE-7)*BIT))&0xff];
            case 6: ++cnt[UNIT_SIZE-6][(src[i]>>((UNIT_SIZE-6)*BIT))&0xff];
            case 5: ++cnt[UNIT_SIZE-5][(src[i]>>((UNIT_SIZE-5)*BIT))&0xff];
            case 4: ++cnt[UNIT_SIZE-4][(src[i]>>((UNIT_SIZE-4)*BIT))&0xff];
            case 3: ++cnt[UNIT_SIZE-3][(src[i]>>((UNIT_SIZE-3)*BIT))&0xff];
            case 2: ++cnt[UNIT_SIZE-2][(src[i]>>((UNIT_SIZE-2)*BIT))&0xff];
            case 1: ++cnt[UNIT_SIZE-1][((src[i]>>((UNIT_SIZE-1)*BIT))^sign_mask)&0xff];
        }
    }
    for(unsigned i=1;i<(1u<<BIT);++i)//Adding
    {
        switch(UNIT_SIZE)
        {
            case 16: cnt[15][i]+=cnt[15][i-1];
            case 15: cnt[14][i]+=cnt[14][i-1];
            case 14: cnt[13][i]+=cnt[13][i-1];
            case 13: cnt[12][i]+=cnt[12][i-1];
            case 12: cnt[11][i]+=cnt[11][i-1];
            case 11: cnt[10][i]+=cnt[10][i-1];
            case 10: cnt [9][i]+=cnt[ 9][i-1];
            case  9: cnt [8][i]+=cnt[ 8][i-1];
            case  8: cnt [7][i]+=cnt[ 7][i-1];
            case  7: cnt [6][i]+=cnt[ 6][i-1];
            case  6: cnt [5][i]+=cnt[ 5][i-1];
            case  5: cnt [4][i]+=cnt[ 4][i-1];
            case  4: cnt [3][i]+=cnt[ 3][i-1];
            case  3: cnt [2][i]+=cnt[ 2][i-1];
            case  2: cnt [1][i]+=cnt[ 1][i-1];
            case  1: cnt [0][i]+=cnt[ 0][i-1];
        }
    }
    for(size_t radix=0;radix<UNIT_SIZE-1;++radix)
    {
        for(size_t i=nmemb;i--;)
            dst[--cnt[radix][(src[i]>>(radix*BIT))&0xff]]=src[i];
        swap_64(&src,&dst);
    }
    //deal with signed value
    for(size_t i=nmemb;i--;)
        dst[--cnt[UNIT_SIZE-1][((src[i]>>((UNIT_SIZE-1)*BIT))^sign_mask)&0xff]]=src[i];
    if(UNIT_SIZE%2)
        memcpy(base,aux,nmemb*sizeof(sort_t));
    free(aux);
    return base;
    #undef BIT
    #undef UNIT_SIZE
}

sort_t *radixSort_32(sort_t *base, const size_t nmemb)
{
    if(nmemb==1)
        return base;
    const unsigned MAX_BIT=8;
    const unsigned BIT=(MAX_BIT<=sizeof(sort_t)*CHAR_BIT)?MAX_BIT:(unsigned)sizeof(sort_t)*CHAR_BIT;
    assert(isExpOf2(BIT)&&isExpOf2(sizeof(sort_t)));
    sort_t *aux = (sort_t*)malloc(nmemb*sizeof(sort_t));
    sort_t *src=base,*dst=aux;
    size_t cnt[1u<<BIT];
    if(!(aux&&nmemb>0))
        return NULL;
    for(size_t radix=0;radix<sizeof(sort_t)*(size_t)CHAR_BIT/(size_t)BIT;++radix)
    {
        const unsigned mask=~(~0u<<BIT);
        memset(cnt,0,sizeof(cnt));
        for(size_t i=nmemb;i--;)
            ++cnt[(src[i]>>(radix*BIT))&mask];
        for(unsigned i=1;i<(1u<<BIT);++i)
            cnt[i]+=cnt[i-1];
        for(size_t i=nmemb;i--;)
            dst[--cnt[(src[i]>>(radix*BIT))&mask]]=src[i];
        swap_64(&src,&dst);
    }
    if(ISFLOAT(sort_t))
    {
        
    }
    else if(ISSIGNED(sort_t))
    {
        const sort_t mask=(sort_t)1<<(sizeof(sort_t)*CHAR_BIT-1);
        size_t i,m,r;
        for(i=-1,r=nmemb,m=nmemb/2;i!=r-1;m=(i+r)/2)
            if(src[m]&mask)
                r=m;
            else
                i=m;
        ++i;
        memcpy(dst,src+i,(nmemb-i)*sizeof(sort_t));
        memcpy(dst+nmemb-i,src,i*sizeof(sort_t));
        if(!(sizeof(sort_t)%2))
            memcpy(src,dst,nmemb*sizeof(sort_t));
    }
    else if(sizeof(sort_t)*CHAR_BIT==BIT)
        memcpy(base,aux,nmemb*sizeof(sort_t));
    free(aux);
    return base;
}

static inline sort_t *partial_radixsort(sort_t *base, const size_t nmemb, sort_t *aux)
{
    #define BIT 8
    #define UNIT_SIZE sizeof(sort_t)
    if(nmemb<2)
        return base;
    sort_t *src=base,*dst=aux;
    size_t cnt[UNIT_SIZE][1<<BIT];
    const uint8_t sign_mask=(uint8_t)ISSIGNED(sort_t)<<7;
    memset(cnt,0,sizeof(cnt));
    for(size_t i=nmemb;i--;)//Counting
    {
        switch(UNIT_SIZE)
        {
            case 16: ++cnt[UNIT_SIZE-16][(src[i]>>((UNIT_SIZE-16)*BIT))&0xff];
            case 15: ++cnt[UNIT_SIZE-15][(src[i]>>((UNIT_SIZE-15)*BIT))&0xff];
            case 14: ++cnt[UNIT_SIZE-14][(src[i]>>((UNIT_SIZE-14)*BIT))&0xff];
            case 13: ++cnt[UNIT_SIZE-13][(src[i]>>((UNIT_SIZE-13)*BIT))&0xff];
            case 12: ++cnt[UNIT_SIZE-12][(src[i]>>((UNIT_SIZE-12)*BIT))&0xff];
            case 11: ++cnt[UNIT_SIZE-11][(src[i]>>((UNIT_SIZE-11)*BIT))&0xff];
            case 10: ++cnt[UNIT_SIZE-10][(src[i]>>((UNIT_SIZE-10)*BIT))&0xff];
            case 9: ++cnt[UNIT_SIZE-9][(src[i]>>((UNIT_SIZE-9)*BIT))&0xff];
            case 8: ++cnt[UNIT_SIZE-8][(src[i]>>((UNIT_SIZE-8)*BIT))&0xff];
            case 7: ++cnt[UNIT_SIZE-7][(src[i]>>((UNIT_SIZE-7)*BIT))&0xff];
            case 6: ++cnt[UNIT_SIZE-6][(src[i]>>((UNIT_SIZE-6)*BIT))&0xff];
            case 5: ++cnt[UNIT_SIZE-5][(src[i]>>((UNIT_SIZE-5)*BIT))&0xff];
            case 4: ++cnt[UNIT_SIZE-4][(src[i]>>((UNIT_SIZE-4)*BIT))&0xff];
            case 3: ++cnt[UNIT_SIZE-3][(src[i]>>((UNIT_SIZE-3)*BIT))&0xff];
            case 2: ++cnt[UNIT_SIZE-2][(src[i]>>((UNIT_SIZE-2)*BIT))&0xff];
            case 1: ++cnt[UNIT_SIZE-1][((src[i]>>((UNIT_SIZE-1)*BIT))^sign_mask)&0xff];
        }
    }
    for(unsigned i=1;i<(1u<<BIT);++i)//Adding
    {
        switch(sizeof(sort_t)*CHAR_BIT/BIT)
        {
            case 16: cnt[15][i]+=cnt[15][i-1];
            case 15: cnt[14][i]+=cnt[14][i-1];
            case 14: cnt[13][i]+=cnt[13][i-1];
            case 13: cnt[12][i]+=cnt[12][i-1];
            case 12: cnt[11][i]+=cnt[11][i-1];
            case 11: cnt[10][i]+=cnt[10][i-1];
            case 10: cnt[9][i]+=cnt[9][i-1];
            case 9: cnt[8][i]+=cnt[8][i-1];
            case 8: cnt[7][i]+=cnt[7][i-1];
            case 7: cnt[6][i]+=cnt[6][i-1];
            case 6: cnt[5][i]+=cnt[5][i-1];
            case 5: cnt[4][i]+=cnt[4][i-1];
            case 4: cnt[3][i]+=cnt[3][i-1];
            case 3: cnt[2][i]+=cnt[2][i-1];
            case 2: cnt[1][i]+=cnt[1][i-1];
            case 1: cnt[0][i]+=cnt[0][i-1];
        }
    }
    for(size_t radix=0;radix<UNIT_SIZE-1;++radix)
    {
        for(size_t i=nmemb;i--;)
            dst[--cnt[radix][(src[i]>>(radix*BIT))&0xff]]=src[i];
        swap_64(&src,&dst);
    }
    //deal with signed value
    for(size_t i=nmemb;i--;)
        dst[--cnt[UNIT_SIZE-1][((src[i]>>((UNIT_SIZE-1)*BIT))^sign_mask)&0xff]]=src[i];
    if(UNIT_SIZE%2)
        memcpy(base,aux,nmemb*UNIT_SIZE);
    return base;
    #undef BIT
}

sort_t *hybridSort(sort_t *base, size_t nmemb,  const unsigned MAX_RAM_MB)
{
    #define q_swap(a,b) {sort_t tmp = a; a = b; b = tmp;}
    #define PACK(B,N,A) {pack.base=B,pack.nmemb=N,pack.extra=A;}
    #define MIXSORT partial_radixsort
    #define STACK_SIZE (CHAR_BIT*sizeof(size_t))
    if(nmemb<2) return base;
    const unsigned long MAX_THRESH=MAX_RAM_MB?MIN(MAX_RAM_MB*1024*1024/sizeof(sort_t),nmemb):nmemb;
    sort_t *aux=(sort_t*)malloc(MAX_THRESH*sizeof(sort_t));
    if(!aux) return NULL;
    struct Layer
    {
        sort_t *base;
        size_t nmemb;
    } stack[STACK_SIZE], *layer;
    layer=stack;
    sort_t *pvt=base+nmemb/2, *l=base, *r=base+nmemb-1;
    do
    {
        if(*l>*pvt)         //inline 3-element insertion sort
            q_swap(*l,*pvt);
        if(*pvt>*r)
        {
            q_swap(*pvt,*r);
            if(*l>*pvt)
                q_swap(*l,*pvt);
        }
        while(l<r)
        {
            for(;*l<*pvt;++l);
            for(;*r>*pvt;--r);
            q_swap(*l,*r);
            if(l==pvt)
            {
                pvt=r;
                ++l;
            }
            else if(r==pvt)
            {
                pvt=l;
                --r;
            }
            else
                ++l,--r;
        }
        switch(((pvt-base < MAX_THRESH)<<1)|(base+nmemb-1-pvt < MAX_THRESH))
        {
            case 0:
                layer->base=base, layer->nmemb=pvt-base;
                ++layer;
                nmemb=base+nmemb-1-pvt;
                base=pvt+1;
                break;
            case 1:
                MIXSORT(pvt+1,base+nmemb-1-pvt,aux);
                nmemb=pvt-base;
                break;
            case 2:
                MIXSORT(base, pvt-base, aux);
                nmemb=base+nmemb-1-pvt;
                base=pvt+1;
                break;
            case 3:
                MIXSORT(pvt+1,base+nmemb-1-pvt,aux);
                MIXSORT(base,pvt-base,aux);
                --layer;
                base=layer->base, nmemb=layer->nmemb;
        }
        l=base, r=base+nmemb-1, pvt=base+nmemb/2;
    }while(layer>=stack);
    free(aux);
    return base;
    #undef q_swap
    #undef STACK_SIZE
    #undef MIXSORT
    #undef PACK
}

void *partial_rsortX(void *base, const size_t nmemb,void *aux, const size_t size, const num_type_t type)
{
    #define BIT 8
    if(nmemb<2) return base;
    uint8_t *src=(uint8_t*)base, *dst=(uint8_t*)aux;
    size_t cnt[256u];
    for(size_t radix=0;radix<size;++radix)
    {
        memset(cnt,0,sizeof(cnt));
        for(size_t i=nmemb;i--;)
            ++cnt[src[i*size+radix]];
        for(unsigned i=1;i < 256u;++i)
            cnt[i]+=cnt[i-1];
        for(size_t i=nmemb;i--;)
            memcpy(dst+--cnt[src[i*size+radix]]*size, src+i*size, size);
        swap(&src,&dst,sizeof(void*));
    }
    if(type>=INT8)//Rearrange for signed values
    {
        const uint8_t mask=0x80;
        size_t i,m,r;
        for(i=-1,r=nmemb,m=nmemb/2;i!=r-1;m=(i+r)/2)
            if(src[(m+1)*size-1]&mask)
                r=m;
            else
                i=m;
        ++i;
        if(type>INT128)
            for(uint8_t *s=src+(nmemb-1)*size,*d=dst;d < dst+(nmemb-i)*size;s-=size,d+=size)
                memcpy(d,s,size);
        else
            memcpy(dst,src+i*size,(nmemb-i)*size);
        memcpy(dst+(nmemb-i)*size,src,i*size);
    }
    if((size+(type>=INT8))%2)
        memcpy(base,aux,nmemb*size);
    return base;
    #undef BIT
}

void *partial_rsortXX(void *base, const size_t nmemb,void *aux, const size_t size, const num_type_t type)
{
    #define BIT 8
    if(nmemb<2) return base;
    uint8_t *src=(uint8_t*)base, *dst=(uint8_t*)aux;
    size_t cnt[256u];
    for(size_t radix=0;radix<size;++radix)
    {
        memset(cnt,0,sizeof(cnt));
        for(size_t i=nmemb;i--;)
            ++cnt[src[i*size+radix]];
        for(unsigned i=1;i < 256u;++i)
            cnt[i]+=cnt[i-1];
        for(size_t i=nmemb;i--;)
            memcpy(dst+--cnt[src[i*size+radix]]*size, src+i*size, size);
        swap(&src,&dst,sizeof(void*));
    }
    if(type>=INT8)//Rearrange for signed values
    {
        const uint8_t mask=0x80;
        size_t i,m,r;
        for(i=-1,r=nmemb,m=nmemb/2;i!=r-1;m=(i+r)/2)
            if(src[(m+1)*size-1]&mask)
                r=m;
            else
                i=m;
        ++i;
        if(type>INT128)
            for(uint8_t *s=src+(nmemb-1)*size,*d=dst;d < dst+(nmemb-i)*size;s-=size,d+=size)
                memcpy(d,s,size);
        else
            memcpy(dst,src+i*size,(nmemb-i)*size);
        memcpy(dst+(nmemb-i)*size,src,i*size);
    }
    if((size+(type>=INT8))%2)
        memcpy(base,aux,nmemb*size);
    return base;
    #undef BIT
}
