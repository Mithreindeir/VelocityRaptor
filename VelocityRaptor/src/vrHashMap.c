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
#include "../include/velocityraptor.h"
#include "../include/vrLinkedList.h"


vrHashTable * vrHashTableAlloc()
{
	return vrAlloc(sizeof(vrHashTable));
}

vrHashTable * vrHashTableInit(vrHashTable * table, int size)
{
	table->buckets = vrArrayInit(vrArrayAlloc(), sizeof(vrLinkedList));
	int prime_size = getPrime(size);
	
	for (int i = 0; i < prime_size; i++)
	{
		vrArrayPush(table->buckets, vrLinkedListInit(vrLinkedListAlloc()));

	}
	table->hash = &vrHashFuncDefault;
	table->deleteFunc = NULL;
	return table;
}

vrHashEntry * vrHashTableLookup(vrHashTable * table, unsigned int key)
{
	vrHashValue hashv = table->hash(key);
	int index = hashv%table->buckets->sizeof_active;

	vrLinkedList* list = table->buckets->data[index];
	vrNode* node = list->head;
	while(node)
	{
		if (key == ((vrHashEntry*)node->data)->key)
			return ((vrHashEntry*)node->data);
		node = node->next;
	}
	return NULL;

}

void vrHashTableInsert(vrHashTable * table, vrHashEntry * entry, unsigned int key)
{

	vrHashValue hashv = table->hash(key);
	int index = hashv%table->buckets->sizeof_active;


	vrLinkedList* list = table->buckets->data[index];
	list->deleteFunc = table->deleteFunc;
	vrNode* node = list->head;
	while (node)
	{
		if (((vrHashEntry*)node->data)->key == key)
		{
			return;
			vrFree(((vrHashEntry*)node->data)->data);
			vrFree(((vrHashEntry*)node->data));
			vrLinkedListRemove(table->buckets->data[index], node);
		}
		node = node->next;
	}
	vrNode* nnode = vrAlloc(sizeof(vrNode));
	nnode->data = entry;
	vrLinkedListAddBack(table->buckets->data[index], nnode);
}

void vrHashTableRemove(vrHashTable * table, unsigned int key)
{
	vrHashValue hashv = table->hash(key);
	int index = hashv%table->buckets->sizeof_active;

	vrLinkedList* list = table->buckets->data[index];
	vrNode* node = list->head;
	while (node)
	{
		if (((vrHashEntry*)node->data)->key == key)
		{
			if(((vrHashEntry*)node->data)->data) vrFree(((vrHashEntry*)node->data)->data);
			if(((vrHashEntry*)node->data)) vrFree(((vrHashEntry*)node->data));
			vrLinkedListRemove(table->buckets->data[index], node);
			return;
		}
		node = node->next;
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
