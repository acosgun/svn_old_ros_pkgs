#include "ClientSocket.h"
#include "SocketException.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;

ClientSocket::ClientSocket (std::string host, int port)
{
  if (!Socket::create ())
    {
      throw SocketException ("Could not create client socket.");
    }

  if (!Socket::connect (host, port))
    {
      throw SocketException ("Could not bind to port.");
    }

}


void ClientSocket::closeManual()
{
  //Socket::~Socket();
}


const ClientSocket &
ClientSocket::operator << (const std::string & s) const
{
  if (!Socket::send (s))
    {
      throw SocketException ("Could not write to socket.");
    }

  return *this;

}

const ClientSocket &
ClientSocket::operator >> (std::string & s) const
{
  if (!Socket::recv (s))
    {
      throw SocketException ("Could not read from socket.");
    }

  return *this;
}

//Allows for integer transfer
const ClientSocket &
ClientSocket::operator >> (unsigned int &i) const
{
 // if (!Socket::recv (i))
 //   {
 //     throw SocketException ("Could not read from socket.");
 //   }

  return *this;
}

