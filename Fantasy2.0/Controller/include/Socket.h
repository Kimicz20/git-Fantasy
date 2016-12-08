#ifndef Socket_class
#define Socket_class

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "SocketException.h"

const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;
const int PACK_SIZE = 1024*512;

class Socket
{
 public:
  Socket();
  virtual ~Socket();

  // 服务器初始化
  bool create();
  bool bind ( const int port );
  bool listen() const;
  bool accept ( Socket& ) const;

  // 数据传输
  int recv ( std::string& ) const;
    //从文件传输
  std::string recvWithFile()const;
  bool send ( const std::string ) const;

  void set_non_blocking ( const bool );

  bool is_valid() const { return sockfd != -1; }

 private:

  int sockfd;//socket文件标志
  struct sockaddr_in server_addr;//socket地址
};


#endif
