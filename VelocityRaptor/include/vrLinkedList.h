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
