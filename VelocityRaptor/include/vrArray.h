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

#ifndef HEADER_VRARRAY
#define HEADER_VRARRAY

//Note: You must deallocate/free
//all memory for objects yourself
typedef struct vrArray
{
	void** data;
	int sizeof_active;
	int sizeof_data;
	int sizeof_array;
	int size_available;
} vrArray;

vrArray* vrArrayAlloc();
vrArray* vrArrayInit(vrArray* arr, int sizeofdata);
void vrArrayDestroy(vrArray* arr);
void vrArrayPush(vrArray* arr, void* object);
void vrArrayPop(vrArray* arr);
void vrArrayReserve(vrArray* arr, int size);
void vrArrayErase(vrArray* arr, int index);
void vrArrayCopy(vrArray* dest, vrArray* src);
void vrArrayClear(vrArray* arr);

#endif
