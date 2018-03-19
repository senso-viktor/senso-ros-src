// using standard exceptions
#include <iostream>
#include <exception>


class Can_exception: public std::exception
{
	virtual const char* what() const throw()
	  {

	    return ("error can_interface: " +  err).c_str();
	  }
public:
	Can_exception(std::string err);

private:
  std::string err;
};
