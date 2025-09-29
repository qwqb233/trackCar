#ifndef __MEMORY_POOL_H__
#define __MEMORY_POOL_H__

#include <stddef.h>

#ifndef uint8_t
#define uint8_t unsigned char
#endif
#ifndef uint16_t
#define uint16_t unsigned short
#endif
#ifndef uint32_t
#define uint32_t unsigned int
#endif

#define MEMORY_POOL_SIZE 10*1024
#define MEMORY_CHUNK_SIZE 256

typedef uint8_t memoryPoolLock;

extern uint16_t sizeofchunk;
extern size_t malloc_test;


typedef struct memoryheader
{
    void* alloc;
    size_t alloc_size;
    size_t chunk_size;
    size_t current;
    size_t clearup;
    struct memoryheader* next;
} memoryheader_t;

void * malloc_stm32(size_t size);
void free_stm32(void* ptr);
long int memory_usage();
#endif

