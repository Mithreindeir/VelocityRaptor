#ifndef HEADER_VRpool
#define HEADER_VRpool

#include "velocityraptor.h"

typedef struct vrMemoryResource;

typedef struct vrMemoryResource
{
	void* object;
	struct vrMemoryResource* next;
} vrMemoryResource;

typedef struct vrMemoryPool
{
	vrMemoryResource** memory;

	int sizeof_data;
	int sizeof_pool;

	vrMemoryResource* firstAvailable;
	vrMemoryResource* lastAvailable;
} vrMemoryPool;

vrMemoryPool* vrMemoryPoolAlloc();
vrMemoryPool* vrMemoryPoolInit(vrMemoryPool* pool, int sizeofdata, int sizeofpool);
vrMemoryResource* vrMemoryPoolGetMemory(vrMemoryPool* pool);
void vrMemoryPoolvrFreeMemory(vrMemoryPool* pool, vrMemoryResource* memory);
void vrMemoryPoolDestroy(vrMemoryPool* pool);
#endif