// using standard exceptions

#include "can_interface/can_exception.h"


Can_exception::Can_exception(std::string err){
	this->err = err;
}
