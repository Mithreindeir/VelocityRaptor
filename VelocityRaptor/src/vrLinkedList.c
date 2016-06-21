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

#include "../include/vrLinkedList.h"
#include "../include/velocityraptor.h"

vrLinkedList * vrLinkedListAlloc()
{
	return vrAlloc(sizeof(vrLinkedList));
}

vrLinkedList * vrLinkedListInit(vrLinkedList * list)
{
	list->head = NULL;
	list->deleteFunc = NULL;
	return list;
}

void vrLinkedListClear(vrLinkedList * list)
{
	while (list->head != NULL)
	{
		vrLinkedListRemove(list, list->head);
	}
}

void vrLinkedListAddFront(vrLinkedList * list, vrNode * node)
{
	node->next = NULL;
	node->prev = NULL;
	vrNode * h  = list->head;
	while (h)
	{
		if (h->next == NULL) node->prev = h;
		h = h->next;
	}
	node->next = list->head;
	list->head = node;

}

void vrLinkedListAddBack(vrLinkedList * list, vrNode * node)
{

	node->next = NULL;
	node->prev = NULL;
	if (list->head == NULL)
	{
		list->head = node;
	}
	else
	{
		vrNode* current = list->head;
		while(current)
		{
			if(current->next == NULL)
			{
				current->next = node;
				node->prev = current;
				break;
			}
			current = current->next;
		}
	}
	list->head->prev = node;
}

void vrLinkedListRemove(vrLinkedList * list, vrNode * node)
{
	vrNode** current = &list->head;
	vrNode** prev = NULL;

	//Linear search
	while (1)
	{
		if (*current == node)
		{
			if (prev == NULL)
			{
				//Removing the first node
				vrNode* n = list->head->next;
				vrFree(list->head);
				list->head = n;
			}
			else
			{
				vrNode* n = (*current)->next;
				(*prev)->next = (*current)->next;
				vrFree(*current);
				*current = n;
			}
			break;
		}
		*current = (*current)->next;
		prev = current;
	}
}
