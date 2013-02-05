#include "bytewriter.h"

using namespace std;

ByteWriter::ByteWriter() {
  directory.assign("/Users/research/Desktop/lhimages/");
}

ByteWriter::~ByteWriter() {
}

string ByteWriter::writeWithTimestamp(const byte* data, uint64_t size, string ext = "dat") {
  stringstream ss;
  ss << directory << clock.getTime() << "." << ext;
  ofstream fp;
  fp.open(ss.str().c_str(), ios::out | ios::binary);
  fp.write((const char*)data, size);
  fp.close();
  return ss.str();
}

void ByteWriter::setDirectory(string new_directory) {
  directory.assign(new_directory);
}

uint32_t*
ByteWriter::loadDepthImageFromFile(string fn) {
  ifstream fp;
  fp.open(fn.c_str(), ios::in | ios::binary);
  ios::pos_type start = fp.tellg();
  fp.seekg(0, ios::end);
  uint64_t bytes = fp.tellg() - start;
  fp.seekg(0, ios::beg);
  char data[bytes];
  fp.read(reinterpret_cast<char*>(data), bytes);
  
  uint64_t image_elements = bytes / 2;
  uint32_t* dst = new uint32_t[image_elements];
  int j = 0;
  for (int i = 0; i < image_elements; i++) {
    uint8_t hi = static_cast<uint8_t>(data[j++]) & 255;
    uint8_t lo = static_cast<uint8_t>(data[j++]) & 255;
    dst[i] = 0x00000000 & ((hi << 8) | lo);
  }
  return dst;
}

byte*
ByteWriter::loadBitmap(string fn) {
  ifstream fp;
  fp.open(fn.c_str(), ios::in | ios::binary);
  ios::pos_type start = fp.tellg();
  fp.seekg(0, ios::end);
  uint64_t bytes = fp.tellg() - start;
  fp.seekg(0, ios::beg);

  byte* data = new byte[bytes];

  fp.read(reinterpret_cast<char*>(data), bytes);
  
  return data;
}

image_t
ByteWriter::loadRawBinaryFile(string fn) {
  ifstream fp;
  fp.open(fn.c_str(), ios::in | ios::binary);
  ios::pos_type start = fp.tellg();
  fp.seekg(0, ios::end);
  uint64_t bytes = fp.tellg() - start;
  fp.seekg(0, ios::beg);
  
  image_t im;
  im.size = bytes;
  im.data = reinterpret_cast<byte*>(new byte[bytes]);
  
  fp.read(reinterpret_cast<char*>(im.data), bytes);
  
  return im;
}