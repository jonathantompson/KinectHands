//
//  zmqpipe.h
//  KinectHands
//
//  Created by research on 5/17/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef BYTEWRITER_h
#define BYTEWRITER_h
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include "clock.h"

using namespace std;

typedef uint8_t byte;
typedef struct image {
  uint64_t size;  
  byte* data;
} image_t;

class ByteWriter {

public:
  ByteWriter();
  ~ByteWriter();
  string writeWithTimestamp(const byte*, uint64_t, string);
  void setDirectory(string);
  uint32_t* loadDepthImageFromFile(string);
  byte* loadBitmap(string);
  image_t loadRawBinaryFile(string);
  
private:
  string directory;
  Clock clock;
};
#endif
