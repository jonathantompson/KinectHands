//
//  callback_queue_item.h
//
//  Created by Jonathan Tompson on 7/10/12.
//

#ifndef THREADING_CALLBACK_QUEUE_ITEM_HEADER
#define THREADING_CALLBACK_QUEUE_ITEM_HEADER

namespace threading {
  
  // This is simple singly-linked-list item.  Used as a sub-class of queue.
  template <class T>
  struct CallbackQueueItem {
    explicit CallbackQueueItem(T value) {
      next = NULL;
      data = value;
    }
    CallbackQueueItem* next;
    T data;
  };
  
}  // namespace threading

#endif  // THREADING_CALLBACK_QUEUE_ITEM_HEADER
