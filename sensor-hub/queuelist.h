#pragma once
#include <Arduino.h>

typedef struct node {
    node * next; // the next node in the list.
} node;

class queuelist
{
  public:
    queuelist();
    void push(node* n);
    node* pop();
    bool isEmpty() const;
    int count() const;

  private:
    unsigned size;
    node* head;
    node* tail;
};
