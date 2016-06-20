/*
* Copyright (c) 2006-2009 Cormac Grindall (Mithreindeir)
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

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
