//
//  Weather update server in C++
//  Binds PUB socket to tcp://*:5556
//  Publishes random weather updates
//
//  Olivier Chamoux <olivier.chamoux@fr.thalesgroup.com>
//

#include "bytepipe.h"

#ifdef _WIN32
  #ifndef snprintf
    #define snprintf _snprintf
  #endif
  #ifndef random
    #define random rand
  #endif
  #ifndef srandom
    #define srandom srand
  #endif
#endif

#define within(num) (int) ((float) num * random () / (RAND_MAX + 1.0))

int main () {

  BytePipe bp("tcp://*:5557");
  
  //  Initialize random number generator
  srandom ((unsigned) time (NULL));

  while (1) {
    
    int zipcode, temperature, relhumidity;
    
    //  Get values that will fool the boss
    zipcode     = within (100000);
    temperature = within (215) - 80;
    relhumidity = within (50) + 10;
    
    //  Send message to all subscribers
    zmq::message_t message(20);
    snprintf ((char *) message.data(), 20 ,
              "%05d %d %d ", zipcode, temperature, relhumidity);
    
        
  }
  return 0;
}