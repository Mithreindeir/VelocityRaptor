#ifndef HEADER_VRHASHTABLE
#define HEADER_VRHASHTABLE

#include <stdint.h>
#include <string.h>
#include "vrAlignedArray.h"
/*
typedef unsigned int vrHashValue;
typedef vrHashValue*(*vrHashFunc)(const char* key, int len);

typedef struct vrHashEntry
{
	void* data;
	vrHashValue key;
	vrHashEntry* next;
} vrHashEntry;

typedef struct vrHashTable
{
	vrHashFunc hash;
	vrAlignedArray* buckets;
} vrHashTable;

vrHashTable* vrHashTableAlloc(vrHashTable* table);
vrHashTable* vrHashTableInit(vrHashTable* table);
vrHashEntry* vrHashTableLookup(vrHashTable* table, const char* key);
void vrHashTableInsert(vrHashTable* table, vrHashEntry* entry, const char* key);
void vrHashTableRemove(vrHashTable* table, const char* key);
vrHashValue vrHashFuncDefault(const char* key, int len);
*/
#endif