#include "queuelist.h"

queuelist::queuelist()
{
    size = 0;       // set the size of queue to zero.
    head = NULL;    // set the head of the list to point nowhere.
    tail = NULL;    // set the tail of the list to point nowhere.
}

void queuelist::push(node* n)
{
    noInterrupts();
    node* t = tail;
    tail = n;
    tail->next = NULL;
    if (isEmpty())
    {
        head = tail;
    } else {
        t->next = tail;
    }
    size++;
    interrupts();
}

node* queuelist::pop()
{
    if (isEmpty())
    {
        return NULL;
    }
    noInterrupts();
    node* item = head;
    head = head->next;
    size--;
    interrupts();
    return item;
}

bool queuelist::isEmpty() const
{
    return head == NULL;
}

int queuelist::count() const
{
    return size;
}
