//
//  shmem.h
//  
//
//  Created by Murphy Stein on 12/5/11.
//  Copyright (c) 2011 WonderLab LLC. All rights reserved.
//

#ifdef __cplusplus
extern "C" {
#endif
#ifndef _shmem_h
#define _shmem_h


//---------------------------------------------------------
// FRAME CONSTANTS
//---------------------------------------------------------
#define RGB_FRAME_WIDTH 640
#define RGB_FRAME_HEIGHT 480
#define RGB_FRAME_SIZE RGB_FRAME_WIDTH * RGB_FRAME_HEIGHT
#define RGB_BYTES_PER_PIXEL 3
#define RGB_FRAME_BYTES RGB_FRAME_SIZE * RGB_BYTES_PER_PIXEL

#define DEPTH_FRAME_WIDTH 640
#define DEPTH_FRAME_HEIGHT 480
#define DEPTH_FRAME_SIZE DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT
#define DEPTH_BYTES_PER_PIXEL 2
#define DEPTH_FRAME_BYTES DEPTH_FRAME_SIZE * DEPTH_BYTES_PER_PIXEL

#define HAND_FRAME_ELEMENTS 30
#define LEFTHAND_FRAME_BYTES HAND_FRAME_ELEMENTS * 4
#define RIGHTHAND_FRAME_BYTES HAND_FRAME_ELEMENTS * 4

#define RGB_BYTES_OFFSET 0
#define DEPTH_BYTES_OFFSET RGB_FRAME_BYTES
#define LEFTHAND_BYTES_OFFSET DEPTH_BYTES_OFFSET + DEPTH_FRAME_BYTES
#define RIGHTHAND_BYTES_OFFSET LEFTHAND_BYTES_OFFSET + LEFTHAND_FRAME_BYTES

#define LOG_LEVEL 1
#define SHARED_MEMORY_FILENAME "/tmp/openniserver_sharedmemory"

#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>   

//---------------------------------------------------------
// VARIABLES
//---------------------------------------------------------

typedef struct shared_memory_buffer {   
  int  status;
  unsigned char data[];
} SharedBuffer;

int const cTOTAL_SHARED_BYTES =  RGB_FRAME_BYTES + 
                                 DEPTH_FRAME_BYTES + 
                                 LEFTHAND_FRAME_BYTES +
                                 RIGHTHAND_FRAME_BYTES +
                                 sizeof(SharedBuffer);
   


//---------------------------------------------------------
// SHARED FUNCTIONS
//---------------------------------------------------------
int AttachToSharedMemory(void);
//---------------------------------------------------------
// CLIENT FUNCTIONS
//---------------------------------------------------------
int ClientStart(void);
unsigned char* ClientReadRGBFrame(void);
unsigned char* ClientReadDepthFrame(void);
unsigned char* ClientReadLeftHandMatrixFrame(void);
unsigned char* ClientReadRightHandMatrixFrame(void);
int ClientCleanup(void);

//---------------------------------------------------------
// SERVER FUNCTIONS
//---------------------------------------------------------
void ServerStart(void);
int ServerWriteRGBFrame(unsigned char* pix);
int ServerWriteDepthFrame(unsigned char* pix);
int ServerWriteLeftHandMatrixFrame(unsigned char*);
int ServerWriteRightHandMatrixFrame(unsigned char*);
   
void ServerCleanup(void);

#endif

#ifdef __cplusplus
}
#endif

