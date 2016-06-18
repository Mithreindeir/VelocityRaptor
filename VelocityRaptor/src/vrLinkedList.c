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
	vrNode* current = list->head;
	vrNode* prev = NULL;
	//Linear search
	while (1)
	{
		if (current == node)
		{
			if (prev == NULL)
			{
				//Removing the first node
				vrNode* n = list->head->next;
				if (list->deleteFunc) list->deleteFunc(list->head->data);
				vrFree(list->head);
				list->head = n;
			}
			else
			{
				prev->next = current->next;
				if (list->deleteFunc) list->deleteFunc(current->data);
				vrFree(current);
			}
			break;
		}
		current = current->next;
		prev = current;
	}
}
