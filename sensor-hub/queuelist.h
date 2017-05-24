#pragma once
#include <Arduino.h>

/**
 * @brief A queue using a simple linked list
 */
typedef struct node {
    node * next; //!< the next node in the queue.
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
    unsigned size; //!< items in queue
    node* head;    //!< oldest item in queue
    node* tail;    //!< newest item in queue
};
