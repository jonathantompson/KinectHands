#include "zmq/bytepipe.h"
#include "zmq/z_helpers.hpp"

using namespace std;
using namespace zmq;

BytePipe::BytePipe() {
  
}

BytePipe::BytePipe(char* pipe) {
  context = new context_t (1);
  publisher = new socket_t(*context, ZMQ_PUB);
  publisher->bind(pipe);
  cout << "BytePipe: Context initialized" << endl;
}

BytePipe::~BytePipe() {
  delete context;
  delete publisher;
}

void BytePipe::sendWithKey(const string& key, message_t &message) {
  zmq::message_t keymessage(key.size());
  memcpy(keymessage.data(), key.data(), key.size());
  bool rc = publisher->send(keymessage, ZMQ_SNDMORE);
  publisher->send(message);
}

void BytePipe::send(message_t &message) {
  publisher->send(message);
}