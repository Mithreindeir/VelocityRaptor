/*
* Copyright (c) 2016 Cormac Grindall (Mithreindeir)
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* vrFreely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef HEADER_VRHASHTABLE
#define HEADER_VRHASHTABLE

#include <stdint.h>
#include <string.h>
#include "vrArray.h"

typedef unsigned int vrHashValue;
typedef vrHashValue*(*vrHashFunc)(unsigned int key);
typedef void*(*vrDataDeleteFunc)(void* data);

typedef struct vrHashEntry vrHashEntry;
typedef struct vrHashEntry
{
	void* data;
	unsigned int key;
	vrHashEntry* next;
} vrHashEntry;

typedef struct vrHashTable
{
	vrHashFunc hash;
	vrArray* buckets;
	vrDataDeleteFunc deleteFunc;
	vrArray* hashPool;
} vrHashTable;

vrHashTable* vrHashTableAlloc();
vrHashTable* vrHashTableInit(vrHashTable* table, int size);
void* vrHashTableLookup(vrHashTable* table, unsigned int key);
void vrHashTableInsert(vrHashTable* table, void* data,  unsigned int key);
void vrHashTableRemove(vrHashTable* table, const unsigned int key);
vrHashValue vrHashFuncDefault(unsigned int key);
void vrHashTableResize(vrHashTable* table);
unsigned int getPrime(int current_prime);


#endif
