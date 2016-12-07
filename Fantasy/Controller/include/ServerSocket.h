// Definition of the ServerSocket class
#ifndef ServerSocket_class
#define ServerSocket_class

#include "Socket.h"

class ServerSocket : private Socket
{
 private:
 	 ServerSocket *new_sock;
 public:
     ServerSocket ( int port );
     ServerSocket (){};
     virtual ~ServerSocket();
     void receive ( std::string &s);
     std::string receiveWithFile ();
     void send (std::string s);
     void accept ( ServerSocket& );

     std::string serverReceive();
     void sendResult(std::string s);
     void serverSend(std::string content);


};


#endif
