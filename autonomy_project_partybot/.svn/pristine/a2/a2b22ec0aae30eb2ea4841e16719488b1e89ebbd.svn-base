#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.h"

class ClientSocket:private Socket
{
public:

  ClientSocket (std::string host, int port);
    virtual ~ ClientSocket ()
  {
  };

  const ClientSocket & operator << (const std::string &) const;
  const ClientSocket & operator >> (std::string &) const;
  void closeManual ( );
  const ClientSocket & operator >> (unsigned int &) const;
};

#endif

