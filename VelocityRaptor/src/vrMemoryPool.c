#include "../include/vrMemoryPool.h"

vrMemoryPool* vrMemoryPoolAlloc()
{
	return vrAlloc(sizeof(vrMemoryPool));
}

vrMemoryPool * vrMemoryPoolInit(vrMemoryPool * pool, int sizeofdata, int sizeofpool)
{
	pool->sizeof_data = sizeofdata;
	pool->sizeof_pool = sizeofpool;
	pool->memory = calloc(sizeof(vrMemoryResource), sizeofpool);


	for (int i = 0; i < sizeofpool; i++)
	{
		pool->memory[i] = vrAlloc(sizeof(vrMemoryResource));
		pool->memory[i]->object = vrAlloc(sizeofdata);
		pool->memory[i]->next = NULL;
	}
	for (int i = 0; i < sizeofpool - 1; i++)
	{
		pool->memory[i]->next = pool->memory[i + 1];
	}

	pool->firstAvailable = pool->memory[0];
	pool->lastAvailable = pool->memory[sizeofpool - 1];

	return pool;
}

vrMemoryResource* vrMemoryPoolGetMemory(vrMemoryPool* pool)
{
	if (pool->firstAvailable == pool->lastAvailable)
	{
		printf("Memory Pool out of space");
		abort();
	}

	vrMemoryResource* newMem = pool->firstAvailable;
	pool->firstAvailable = pool->firstAvailable->next;

	return newMem;
}

void vrMemoryPoolvrFreeMemory(vrMemoryPool* pool, vrMemoryResource * memory)
{
	pool->lastAvailable->next = memory;

	pool->lastAvailable = pool->lastAvailable->next;
	pool->lastAvailable->next = NULL;
}

void vrMemoryPoolDestroy(vrMemoryPool * pool)
{
	vrFree(pool->memory);
	vrFree(pool);
}
