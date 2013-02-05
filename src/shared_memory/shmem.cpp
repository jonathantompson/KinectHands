#include "shmem.h"

static int   shmem_id;
static key_t shmem_key;
static volatile struct shared_memory_buffer* buffer;

//---------------------------------------------------------
// SHARED FUNCTIONS
//---------------------------------------------------------
int AttachToSharedMemory() {
  if ((shmem_key = ftok(SHARED_MEMORY_FILENAME, 'A')) == -1) {
    printf("Error with ftok\n");
    printf("If this is the client:  Are you sure the server is running?\n");
    exit(1);
  }
  shmem_id = shmget(shmem_key, cTOTAL_SHARED_BYTES, 0644 |IPC_CREAT);
  
  if (shmem_id == -1) {
    printf("Error with shmget.\n");
    printf("Did you allocate enough memory in /etc/sysctl.conf ?\n");
    printf("Or are libraries out of date?\n");
    exit(1);
  }
  buffer = reinterpret_cast<SharedBuffer*>(shmat(shmem_id, 0, 0));
  // buffer->rgb_pixels = (unsigned char*)buffer->data;
  printf("Attached to shared memory.\n");
  return 1;
}

//---------------------------------------------------------
// SERVER FUNCTIONS
//---------------------------------------------------------
void ServerStart(void) {
  FILE* temp = fopen(SHARED_MEMORY_FILENAME, "w"); 
  fclose(temp);
  printf("Created shared memory file at %s.\n", SHARED_MEMORY_FILENAME);
  AttachToSharedMemory();
  buffer->status = 1;
}

int ServerWriteRGBFrame(unsigned char* pix) {
  memcpy(reinterpret_cast<void*>(const_cast<unsigned char*>(buffer->data)), pix,
         RGB_FRAME_BYTES);
  return 1;
}

int ServerWriteDepthFrame(unsigned char* pix) {
  memcpy(reinterpret_cast<void*>(const_cast<unsigned char*>(
                  &buffer->data[DEPTH_BYTES_OFFSET])), pix, DEPTH_FRAME_BYTES);
  return 1;
}

int ServerWriteLeftHandMatrixFrame(unsigned char* vals) {
  memcpy(reinterpret_cast<void*>(const_cast<unsigned char*>(
            &buffer->data[LEFTHAND_BYTES_OFFSET])), vals, LEFTHAND_FRAME_BYTES);
  return 1;
}

int ServerWriteRightHandMatrixFrame(unsigned char* vals) {
  memcpy(reinterpret_cast<void*>(const_cast<unsigned char*>(
          &buffer->data[RIGHTHAND_BYTES_OFFSET])), vals, RIGHTHAND_FRAME_BYTES);
  return 1;
}

void ServerCleanup() {
  if (shmctl(shmem_id, IPC_RMID, NULL) == -1) {
    printf("Error freeing shared memory.");
    exit(1);
  } else {
    remove(SHARED_MEMORY_FILENAME);
    printf("Deleted shared memory file from %s.\n", SHARED_MEMORY_FILENAME);
  }
}

//---------------------------------------------------------
// CLIENT FUNCTIONS
//---------------------------------------------------------
int ClientStart() {    
  AttachToSharedMemory();
  return 1;
}

unsigned char* ClientReadRGBFrame(void) {
  return (unsigned char*)(buffer->data);
}

unsigned char* ClientReadDepthFrame(void) {
  return (unsigned char*)(&buffer->data[DEPTH_BYTES_OFFSET]);
}

unsigned char* ClientReadLeftHandMatrixFrame(void) {
  return (unsigned char*)(&buffer->data[LEFTHAND_BYTES_OFFSET]);
}

unsigned char* ClientReadRightHandMatrixFrame(void) {
  return (unsigned char*)(&buffer->data[RIGHTHAND_BYTES_OFFSET]);
}


int ClientCleanup(void) {  
  if (buffer != NULL) {
    if (shmctl(shmem_id, IPC_RMID, NULL) == -1) {
      printf("Error freeing shared memory.");
      return -1;
    }
  }
  
  return 1;
}
