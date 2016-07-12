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

#include "../include/vrHashMap.h"
#include "../include/velocityraptor.h"


vrHashTable * vrHashTableAlloc()
{
	return vrAlloc(sizeof(vrHashTable));
}

vrHashTable * vrHashTableInit(vrHashTable * table, int size)
{
	table->buckets = vrArrayInit(vrArrayAlloc(), sizeof(vrHashEntry*));
	int prime_size = getPrime(size);
	
	for (int i = 0; i < prime_size; i++)
	{
		vrArrayPush(table->buckets, NULL);
	}
	table->hash = &vrHashFuncDefault;
	table->deleteFunc = NULL;
	table->hashPool = vrArrayInit(vrArrayAlloc(), sizeof(vrHashEntry*));

	for (int i = 0; i < 100; i++)
	{
		vrArrayPush(table->hashPool, vrAlloc(sizeof(vrHashEntry)));
	}
	return table;
}

void * vrHashTableLookup(vrHashTable * table, unsigned int key)
{
	vrHashValue hashv = table->hash(key);
	int index = hashv%table->buckets->sizeof_active;

	vrHashEntry* head = table->buckets->data[index];
	while(head)
	{
		if (key == head->key)
			return head->data;
		head = head->next;
	}
	return NULL;
}

void vrHashTableInsert(vrHashTable * table, void* data, unsigned int key)
{
	vrHashEntry* entry;
	if (table->hashPool->sizeof_active > 0)
	{
		entry = table->hashPool->data[table->hashPool->sizeof_active - 1];
		vrArrayPop(table->hashPool);
	}
	else
	{
		entry = vrAlloc(sizeof(vrHashEntry));
	}
	entry->key = key;
	entry->data = data;
	entry->next = NULL;

	vrHashValue hashv = table->hash(key);
	int index = hashv%table->buckets->sizeof_active;
	vrHashTableRemove(table, key);

	vrHashEntry* head = table->buckets->data[index];
	if (table->buckets->data[index] == NULL)
		table->buckets->data[index] = entry;
	else
	{
		vrHashEntry* current = table->buckets->data[index];
		while (current)
		{
			if (current->next == NULL)
			{
				current->next = entry;
				break;
			}
			current = current->next;
		}
	}

}

void vrHashTableRemove(vrHashTable * table, unsigned int key)
{
	vrHashValue hashv = table->hash(key);
	int index = hashv%table->buckets->sizeof_active;

	while(1)
	{
		vrHashEntry* current = table->buckets->data[index];
		vrHashEntry** prev = &table->buckets->data[index];

		while (current && current->key != key)
		{
			prev = &current->next;
			current = current->next;
		}

		if (current)
		{
			*prev = current->next;
			vrArrayPush(table->hashPool, current);
		}
		if (!current)
			break;
	}

}

vrHashValue vrHashFuncDefault(unsigned int key)
{
	key = ((key >> 16) ^ key) * 0x45d9f3b;
	key = ((key >> 16) ^ key) * 0x45d9f3b;
	key = ((key >> 16) ^ key);
	return key;
}

void vrHashTableResize(vrHashTable * table)
{
	/*
	int newPrime = getPrime(table->buckets->sizeof_array);

	vrHashEntry** buckets = (vrLinkedList**)vrCalloc(sizeof(vrLinkedList*), newPrime);
	for (int i = 0; i < newPrime; i++)
	{
		buckets[i] = vrLinkedListInit(vrLinkedListAlloc());
	}
	int n = 0;
	for (int i = 0; i < table->buckets->sizeof_active; i++)
	{
		if (table->buckets->data[i])
		{

			vrLinkedList* list = table->buckets->data[i];
			vrNode* node = list->head;
			while (node)
			{
				vrHashEntry* entry = ((vrHashEntry*)node->data);

				vrHashValue hashv = table->hash(entry->key);;
				int index = hashv%newPrime;
				vrLinkedListAddBack(buckets[i], node);

				node = node->next;
			}
		}
	}
	for (int i = 0; i < table->buckets->sizeof_active; i++)
	{
		vrFree(table->buckets->data[i]);
	}
	vrFree(table->buckets->data);
	table->buckets->data = buckets;
	table->buckets->sizeof_array = newPrime;
	table->buckets->sizeof_active = newPrime;
	*/
}

unsigned int getPrime(int current_prime)
{
	//If current_prime is not a prime, return first prime
	//Larger than it
	int highest = 0;
	vrBOOL larger = vrFALSE;
	for (int i = 0; i < NUM_PRIMES; i++)
	{
		if (primes[i] == current_prime && i < 25)
		{
			return primes[i + 1];
			
		}
		if(primes[i] > current_prime && !larger)
		{
			highest = primes[i];
			larger = vrTRUE;
		}
	}
	return highest;
}
