//======================================================================
/**
 *  @file
 *  can_interface.h
 *
 *  @section can_interface.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author
 *  @date
 *  
 *  
 *  
 *
 */
//======================================================================


//#ifndef FUNCTIONS_H_
//#define FUNCTIONS_H_


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> 

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <vector>
#include <string>
 
#include <signal.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#include <errno.h>

#include <ros/ros.h>


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------


class Can_interface

{
	public:  
	~Can_interface();
	bool initCAN(std::string dev,std::vector<int> read_can_id, int timeout);
	int closeCAN();
	int writeCAN(can_frame *frame);
	int writeCAN(can_frame *frame,int size);
	int readCAN(can_frame *frame);
	int readCAN(can_frame *frame, int size);
	int isAvailable();

	private:
     bool isActive;
	 int soc; //socket pre zbernicu
};


#ifdef __cplusplus
}
#endif

//#endif /* FUNCTIONS_H_ */
