/*
 * CAN INTERFACE

 */
 

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include "can_interface/can_interface.h"
#include "can_interface/can_exception.h"

//------------------------------------------------------------------------
// Support functions
//------------------------------------------------------x4]
//
bool Can_interface::initCAN(std::string dev, std::vector<int> read_can_id, int timeout){\

		std::string err = "init error - ";
		  soc = socket( PF_CAN, SOCK_RAW, CAN_RAW);
		    if(soc < 0){
		    	err += "socket does not create - " + std::string(strerror(errno));
				ROS_ERROR("%s", err.c_str());
		    	throw Can_exception(err.c_str());
		        return false;
		    }
			    fcntl(soc, F_SETFL, fcntl(soc, F_GETFL) | O_RDWR);
		   // fcntl(soc, F_SETFL, fcntl(soc, F_GETFL) | O_NONBLOCK); //neblokujuci read
		    
		    struct ifreq ifr;
		    strcpy(ifr.ifr_name, dev.c_str());
		   int ret = ioctl(soc, SIOCGIFINDEX, &ifr);  //naco ukladas navratovu hodnotu ked ju nikde okrem podmienky nepouzijes?

		   if(ret != 0){
		        close(soc);
		        err += "ioctl - " + std::string(strerror(errno));
			   ROS_ERROR("%s", err.c_str());

			   throw Can_exception(err.c_str());
		    }
	
		  /* struct can_filter rfilter[read_can_id.size()];

			for (int i = 0; i < read_can_id.size(); i++){
		 		rfilter[i].can_id   = read_can_id[i]; //sem id toho co chces dostavat
		 		rfilter[i].can_mask = ( CAN_EFF_FLAG | CAN_SFF_MASK); //toto nechaj tak
		 	 }

		 	if(setsockopt(soc, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter))){
		 		close(soc);
		 		 err += "setsockopt - " + std::string(strerror(errno));
				ROS_ERROR("%s", err.c_str());

				throw Can_exception(err.c_str());
		 	}*/

		   can_err_mask_t err_mask =
		      ( CAN_ERR_TX_TIMEOUT   /* TX timeout (by netdevice driver) */
		        | CAN_ERR_LOSTARB      /* lost arbitration    / data[0]    */
		        | CAN_ERR_CRTL         /* controller problems / data[1]    */
		        | CAN_ERR_PROT         /* protocol violations / data[2..3] */
		        | CAN_ERR_TRX          /* transoceiver status  / data[4]    */
		        | CAN_ERR_ACK           /* received no ACK on transmission */
		        | CAN_ERR_BUSOFF        /* bus off */
		        | CAN_ERR_BUSERROR      /* bus error (may flood!) */
		        | CAN_ERR_RESTARTED     /* controller restarted */
		 );

		    ret = setsockopt(soc, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,  //naco ukladas navratovu hodnotu ked ju nikde okrem podmienky nepouzijes?
		       &err_mask, sizeof(err_mask));

		    if(ret != 0){
		        close(soc);
		        err += "setsockopt - " + std::string(strerror(errno));
				ROS_ERROR("%s", err.c_str());

				throw Can_exception(err.c_str());
		    }


		    struct sockaddr_can addr = {0};
		    addr.can_family = AF_CAN;
		    addr.can_ifindex = ifr.ifr_ifindex;
		    ret = bind( soc, (struct sockaddr*)&addr, sizeof(addr) ); //naco ukladas navratovu hodnotu ked ju nikde okrem podmienky nepouzijes?

		    if(ret != 0){
				        close(soc);
				        err += "bind - " + std::string(strerror(errno));
				ROS_ERROR("%s", err.c_str());

				throw Can_exception(err.c_str());
				    }

		    if (timeout >= 0){
		  		struct timeval tm;
		  		tm.tv_sec = 0;
		  		tm.tv_usec = timeout * 1000;

		  		if (setsockopt (soc, SOL_SOCKET, SO_RCVTIMEO, (char *)&tm, sizeof(tm)) < 0){
		  			close(soc);
		  			 err += "setsockopt - " + std::string(strerror(errno));
					ROS_ERROR("%s", err.c_str());

					throw Can_exception(err.c_str());
				}
		    }
		    isActive = true;
	  return true;
    }


Can_interface::~Can_interface(){
	if (isActive){
		close(soc);
		}
}

int Can_interface::closeCAN(){
	if (isActive){
		isActive = false;
		return close(soc);
	}
	return 0;
}


int Can_interface::writeCAN(can_frame *frame){
	if (isActive){
		try{
			int ret = write(soc,frame,sizeof(struct can_frame));
					if (ret < 0){
						ROS_ERROR("%s", strerror(errno));
						closeCAN();
						throw Can_exception(strerror(errno));
					}
			return ret;
		} catch(std::exception& e){
			std::string err(e.what());
			ROS_ERROR("%s", err.c_str());

			closeCAN();
			throw Can_exception(("CAN not is ready: " + err).c_str());
		}
		return -1;
	}
	return -1;
}
int Can_interface::writeCAN(can_frame *frame, int size){
	if (isActive){
		try{
			int ret = 0;
			for (int i = 0; i < size; i++){
				ret += write(soc,&frame[i], sizeof(struct can_frame));
					if (ret < 0){
						closeCAN();
						throw Can_exception(strerror(errno));
					}
				}
			return ret;
		} catch(std::exception& e){
			std::string err(e.what());
			closeCAN();
			throw Can_exception(("CAN not is ready: " + err).c_str());
		}
		return -1;
	}
	return -1;
}

int Can_interface::readCAN(can_frame *frame){
	if(isActive){
		return read(soc, frame, sizeof(struct can_frame));
	   }
	return -1;
}

int Can_interface::readCAN(can_frame *frame, int size){
	if(isActive){
		int result = 0;
		for(int i = 0; i < size; i++)
			result += read(soc, &frame[i], sizeof(struct can_frame));
		return result;
	   }
	return -1;
}


int Can_interface::isAvailable ()
{
  if (!isActive) {
    return 0;
  }
  // Is our socket in the return list of readable sockets
  int res = 0;
     fd_set sready;
     struct timeval  nowait;

     try{
     FD_ZERO(&sready);
     FD_SET(soc, &sready);
     //bzero((char *)&nowait,sizeof(nowait));
     memset((char *) &nowait, 0, sizeof(nowait));

     return select(soc + 1, &sready, NULL, NULL, &nowait);
     } catch(std::exception& e){
     			std::string err(e.what());
     			//closeCAN();
     			throw Can_exception(("CAN not is ready: " + err).c_str());
     		}
}
