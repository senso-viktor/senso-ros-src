#include <ros/ros.h>

#include "can_interface/can_interface.h"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>




int main( int argc, char **argv )
{

  ros::init(argc, argv, "wsg_50_can");

   ros::NodeHandle nh("~");
   
   ROS_INFO("WSG 50 - CAN ROS NODE");
    boost::interprocess::shared_memory_object shm_obj(boost::interprocess::open_or_create,"test",boost::interprocess::read_only);
   shm_obj.truncate(sizeof(Can_interface));
   boost::interprocess::mapped_region region(shm_obj, boost::interprocess::read_write, 0, sizeof(Can_interface));
   Can_interface *pokus = (Can_interface *) region.get_address(); 
   
     // pokus->testClass();
   
   /*int descriptor = shm_open("/test", O_RDWR | O_CREAT, 0777);

if (descriptor < 0) {
  
} else {

    ftruncate(descriptor, sizeof(Can_interface));
    Can_interface *pokus = (Can_interface *) mmap(NULL, sizeof(Can_interface), PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED, descriptor, 0);
     pokus->testClass();
 

  // Can_interface pokus = new (ptr) Can_interface();
}*/
   return 0;

}

