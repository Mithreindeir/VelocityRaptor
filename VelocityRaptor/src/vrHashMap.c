#include "../include/vrHashMap.h"
/*
vrHashTable * vrHashTableAlloc(vrHashTable * table)
{
	return malloc(sizeof(vrHashTable));
}

vrHashTable * vrHashTableInit(vrHashTable * table)
{
	table->buckets = vrAlignedArrayInit(vrAlignedArrayAlloc(), sizeof(vrHashEntry));
	return table;
}

vrHashEntry * vrHashTableLookup(vrHashTable * table, const char * key)
{
	
	vrHashValue hashv = table->hash(table, key, strlen(key));
	int index = hashv%table->buckets->sizeof_active;
	if (hashv > table->buckets->sizeof_active) return NULL;

	for (vrHashEntry* entry = ((vrHashEntry*)table->buckets->data[hashv]); entry; entry = entry->next)
	{
		if (!strcmp(key, entry->key))
			return entry;
	}

	return NULL;
	
}

void vrHashTableInsert(vrHashTable * table, vrHashEntry * entry, const char * key)
{

}

void vrHashTableRemove(vrHashTable * table, const char * key)
{
}

vrHashValue vrHashFuncDefault(const char * key, int len)
{
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
*/