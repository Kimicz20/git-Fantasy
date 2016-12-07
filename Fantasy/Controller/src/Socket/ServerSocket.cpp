// Implementation of the ServerSocket class

#include "ServerSocket.h"

using namespace std;

ServerSocket::ServerSocket ( int port )
{

  new_sock = new ServerSocket();

  if ( ! Socket::create() )
    {
      throw SocketException ( "不能创建Socket连接." );
    }

  if ( ! Socket::bind ( port ) )
    {
      throw SocketException ( "不能绑定该端口." );
    }

  if ( ! Socket::listen() )
    {
      throw SocketException ( "不能监听该Socket端口." );
    }
    cout << "。。。等待链接。。。" <<endl;
    accept (*new_sock);
}

ServerSocket::~ServerSocket(){}

void ServerSocket::receive ( std::string& s )
{
  if ( ! Socket::recv ( s ) )
    {
      throw SocketException ( "不能读取该Socket." );
    }
}
void ServerSocket::send (std::string s)
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "不能改写该Socket." );
    }
}

string ServerSocket::receiveWithFile ( )
{
  string result = Socket::recvWithFile();
  if ( result == "" )
    {
      throw SocketException ( "不能读取该Socket." );
  }
  return result;
}

void ServerSocket::accept ( ServerSocket& sock )
{
  if ( ! Socket::accept ( sock ) )
    {
      throw SocketException ( "不能建立该Socket." );
    }
}

void ServerSocket::sendResult(std::string s){
    try
    {
      new_sock->send(s);
    }
    catch ( SocketException& e )
    {
      cout << "Exception was serversocket caught:" << e.description() << "\nExiting.\n";
    }
}

void ServerSocket::serverSend(std::string content)
{
   try
    {
       new_sock->send(content);
    }
  catch ( SocketException& e )
    {
      cout << "Exception was serversocket caught:" << e.description() << "\nExiting.\n";
    }
}

string ServerSocket::serverReceive(){
   try
    {
      return new_sock->receiveWithFile();
    }
  catch ( SocketException& e )
    {
      cout << "Exception was serversocket caught:" << e.description() << "\nExiting.\n";
    }
}
