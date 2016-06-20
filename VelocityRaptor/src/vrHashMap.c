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

#include "../include/vrHashMap.h"

vrHashTable * vrHashTableAlloc()
{
	return vrAlloc(sizeof(vrHashTable));
}

vrHashTable * vrHashTableInit(vrHashTable * table)
{
	table->buckets = vrAlignedArrayInit(vrAlignedArrayAlloc(), sizeof(vrHashEntry));
	table->buckets;
	for (int i = 0; i < 100; i++)
	{
		vrAlignedArrayPush(table->buckets, NULL);
	}
	table->hash = &vrHashFuncDefault;
	return table;
}

vrHashEntry * vrHashTableLookup(vrHashTable * table, const char * key)
{
	vrHashValue hashv = table->hash(key, strlen(key));
	int index = hashv%table->buckets->sizeof_active;
	if (index > table->buckets->sizeof_active) return NULL;
	vrHashEntry* entry = ((vrHashEntry*)table->buckets->data[index]);


	for (vrHashEntry* entry = ((vrHashEntry*)table->buckets->data[index]); entry; entry = entry->next)
	{
		if (!strcmp(key, entry->key))
			return entry;
	}
	return NULL;

}

void vrHashTableInsert(vrHashTable * table, vrHashEntry * entry, const char * key)
{

	vrHashValue hashv = table->hash(key, strlen(key));
	int index = hashv%table->buckets->sizeof_active;
	if (index > table->buckets->sizeof_array) return;

	vrHashEntry** head = &table->buckets->data[index];
	*head = entry;
	while (*head)
		head = &(*head)->next;

	*head = entry;
}

void vrHashTableRemove(vrHashTable * table, const char * key)
{
}

vrHashValue vrHashFuncDefault(const char * key, int len)
{
	len = 6;
	//Simple Bob Jenkins hash, from wikipedia
	vrHashValue hash = 0;
	for (int i = 0; i < len; i++)
	{
		hash += key[i];
		hash += (hash << 10);
		hash ^= (hash >> 6);
	}
	hash += (hash << 3);
	hash ^= (hash >> 11);
	hash += (hash << 15);


	return hash;
}
