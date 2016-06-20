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

#ifndef HEADER_VRLINKEDLIST
#define HEADER_VRLINKEDLIST

typedef void*(*vrNodeDeleteFunc)(void* node);

typedef struct vrNode vrNode;
typedef struct vrNode
{
	void* data;
	vrNode * next;
	vrNode * prev;
} vrNode;

//Doubly linked list
typedef struct vrLinkedList
{
	vrNode* head;
	vrNodeDeleteFunc deleteFunc;
} vrLinkedList;

vrLinkedList* vrLinkedListAlloc();
vrLinkedList* vrLinkedListInit(vrLinkedList* list);
void vrLinkedListClear(vrLinkedList* list);
void vrLinkedListAddFront(vrLinkedList* list, vrNode* node);
void vrLinkedListAddBack(vrLinkedList* list, vrNode* node);
void vrLinkedListRemove(vrLinkedList* list, vrNode* node);

#endif
