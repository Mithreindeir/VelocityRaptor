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


vrHashTable * vrHashTableAlloc()
{
	return vrAlloc(sizeof(vrHashTable));
}

vrHashTable * vrHashTableInit(vrHashTable * table, int size)
{
	table->buckets = vrArrayInit(vrArrayAlloc(), sizeof(vrHashEntry));
	int prime_size = getPrime(size);
	
	for (int i = 0; i < prime_size; i++)
	{
		vrArrayPush(table->buckets, NULL);
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

	vrHashEntry** head = &table->buckets->data[index];

	while (*head)
		head = &(*head)->next;

	entry->next = NULL;
	*head = entry;
}

void vrHashTableRemove(vrHashTable * table, const char * key)
{
	vrHashValue hashv = table->hash(key, strlen(key));
	int index = hashv%table->buckets->sizeof_active;

	vrHashEntry** current = &table->buckets->data[index];
	vrHashEntry** prev = NULL;

	while (1)
	{
		if (!strcmp(key, (*current)->key))
		{
			if (prev == NULL)
			{
				vrHashEntry* n = (*current)->next;

				vrFree((*current)->data);

				vrFree(*current);
				*current = n;
			}
			else
			{
				vrHashEntry* n = (*current)->next;

				(*prev)->next = (*current)->next;
				vrFree((*current)->data);
				vrFree(*current);
				*current = n;
			}
			break;
		}
		*current = (*current)->next;
		prev = current;
	}
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

void vrHashTableResize(vrHashTable * table)
{
	int newPrime = getPrime(table->buckets->sizeof_array);
	printf("%d and \n", newPrime);
	vrHashEntry** buckets = (vrHashEntry**)vrCalloc(sizeof(vrHashEntry*), newPrime);

	int n = 0;
	for (int i = 0; i < table->buckets->sizeof_active; i++)
	{
		if (table->buckets->data[i])
		{
			for (vrHashEntry* entry = ((vrHashEntry*)table->buckets->data[i]); entry; entry = entry->next)
			{
				vrHashValue hashv = table->hash(entry->key, strlen(entry->key));
				int index = hashv%newPrime;

				vrHashEntry** head = &buckets[index];

				while (*head)
					head = &(*head)->next;

				entry->next = NULL;
				*head = entry;
			}

		}
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
