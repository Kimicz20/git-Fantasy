
#include "Socket.h"
using namespace std;

//通过初始化列表来出书画
Socket::Socket():sockfd ( -1 )
{
  memset (&server_addr,0,sizeof( server_addr ));
}

Socket::~Socket()
{
  if ( is_valid() )
    ::close ( sockfd );
}

bool Socket::create()
{
  sockfd = socket ( AF_INET,SOCK_STREAM,0 );
  if ( ! is_valid() ){
    cout<<"Socket error:\n";
    return false;
  }

  // TIME_WAIT - argh
  int on = 1;
  if ( setsockopt ( sockfd, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 )
    return false;
  return true;
}


bool Socket::bind ( const int port )
{

  if ( ! is_valid() )
    {
      return false;
    }
  /* 服务器端填充 sockaddr结构  */
  bzero(&server_addr,sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons ( port );

  int bind_return = ::bind ( sockfd,
			     ( struct sockaddr * ) (&server_addr),
			     sizeof ( server_addr ) );

  if ( bind_return == -1 )
    {
      cout<<"Bind error!\n";
      return false;
    }

  return true;
}


bool Socket::listen() const
{
  if ( ! is_valid() )
    {
      return false;
    }

  int listen_return = ::listen ( sockfd, MAXCONNECTIONS );

  if ( listen_return == -1 )
    {
    	cout<<"Listen error\n";
      return false;
    }

  return true;
}


bool Socket::accept ( Socket& new_socket ) const
{
  int addr_length = sizeof (struct sockaddr_in);
  new_socket.sockfd = ::accept ( sockfd, ( sockaddr * ) &server_addr, ( socklen_t * ) &addr_length );

  if ( new_socket.sockfd <= 0 ){
    cout<<"Accept error\n";
    return false;
  }
  else{
    cout<<"...客户端已经连接... "<<inet_ntoa(server_addr.sin_addr)<<"\n";
    return true;
  }
}


bool Socket::send ( const string s ) const
{
  int status = ::send ( sockfd, s.c_str(), s.size(), 0 );
  if ( status == -1 )
    {
      return false;
    }
  else
    {
      return true;
    }
}



int Socket::recv ( string& s ) const
{
  char buf [ MAXRECV + 1 ];

  s = "";

  memset ( buf, 0, MAXRECV + 1 );

  int status = ::recv ( sockfd, buf, MAXRECV, 0 );

  if ( status == -1 )
    {
      cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
      return 0;
    }
  else if ( status == 0 )
    {
      return 0;
    }
  else
    {
      s = buf;
      return status;
    }
}

string Socket::recvWithFile()const
{
    string file_name="";
    unsigned long file_len  = 0;
    long int read_size = 0;
    long int rlen =0;
    bool wirteFlag = false;
    FILE* pf = NULL;

     while ( true )
     {
        // cout<<"\n\t....传输线程 ( "<<pthread_self()<<" )处理....\n";
        string str;
        Socket::recv(str);
        rlen = str.size();
        string type = str.substr(0,9);
        if(str == "exit"){
          break;
        }else if(type == "fileName:"){
          file_name = str.substr(9,rlen);
        }else if(type == "fileSize:"){
          file_len = atol(str.substr(9,rlen).c_str());
        }else{
            // 写入文件;
           if(!wirteFlag){
              wirteFlag = true;

              //获取程序路径
              char buf[ 1024 ];
              getcwd(buf, 1024);
              string fulldir(buf);
              //拼接文件完全路径
              string tmp = "/Controller";
              fulldir = fulldir.substr(0,fulldir.length()-tmp.length());
              string fullPath = fulldir + "/" + file_name;
              pf = fopen(fullPath.c_str(), "wb+");
              if(pf == NULL)
             {
                cout<<"Open file error!\n";
                close(sockfd);
                break;
              }
           }
            int wn = fwrite(str.c_str(), sizeof(char), rlen, pf);
            read_size += rlen;
          if(read_size >= file_len) {
            // cout<<"Receive Over\nFile Name = "<<file_name<<", File len = "<<file_len<<" , Already read size = "<<read_size<<endl;
            cout<<"...关闭连接..."<<endl;
            /* 这个通讯已经结束 */
            fclose(pf);
            break;
          }
        }
     }
  return file_name;
}

void Socket::set_non_blocking ( const bool b )
{
  int opts;
  opts = fcntl ( sockfd,F_GETFL );
  if ( opts < 0 )
    {
      return;
    }

  if ( b )
    opts = ( opts | O_NONBLOCK );
  else
    opts = ( opts & ~O_NONBLOCK );
  fcntl ( sockfd,F_SETFL,opts );
}
