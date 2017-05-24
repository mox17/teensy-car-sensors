#include "queuelist.h"

queuelist::queuelist()
{
    size = 0;
    head = NULL;
    tail = NULL;
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
