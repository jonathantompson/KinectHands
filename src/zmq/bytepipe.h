//
//  zmqpipe.h
//  KinectHands
//
//  Created by research on 5/17/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef KinectHands_zmqpipe_h
#define KinectHands_zmqpipe_h
#include <iostream>
#include <zmq.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

using namespace zmq;
using namespace std;

class BytePipe {
  
public:
  BytePipe();
  BytePipe(char*);
  ~BytePipe();
  void send(message_t&);
  void sendWithKey(const string&, message_t&);

private:
  context_t* context;
  socket_t* publisher;

};
#endif
