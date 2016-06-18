#ifndef HEADER_VRALIGNEDARRAY
#define HEADER_VRALIGNEDARRAY

#include "velocityraptor.h"

typedef struct vrAlignedArray
{
	void** data;
	int sizeof_active;
	int sizeof_data;
	int sizeof_array;
	int size_available;
} vrAlignedArray;

vrAlignedArray* vrAlignedArrayAlloc();
vrAlignedArray* vrAlignedArrayInit(vrAlignedArray* arr, int sizeofdata);
void vrAlignedArrayPush(vrAlignedArray* arr, void* object);
void vrAlignedArrayPop(vrAlignedArray* arr);
void vrAlignedArrayReserve(vrAlignedArray* arr, int size);
void vrAlignedArrayErase(vrAlignedArray* arr, int index);

#endif