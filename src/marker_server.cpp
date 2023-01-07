#include <ros/ros.h>
#include <EXPROBLAB_Assignment2/RoomConnection.h>
#include <EXPROBLAB_Assignment2/RoomInformation.h>

bool markerCallback(EXPROBLAB_Assignment2::RoomInformation::Request &req, EXPROBLAB_Assignment2::RoomInformation::Response &res){
/**
 * \brief This function provides information about a room in a building based on an input ID.
 *
 * The function processes the request by looking at the value of req.id and setting the appropriate
 * values in the response message. If the ID does not correspond to any known room, the function sets
 * res.room to "no room associated with this marker id".
 *
 * \param
 * 	req The request message containing the ID of the room
 * 	res The response message to be filled with information about the room
 *
 * \return 
 * 	True if the request was successful, false otherwise
 */
	EXPROBLAB_Assignment2::RoomConnection conn;
	switch (req.id){
	case 11:
		res.room = "E";
		res.x = 1.5;
		res.y = 8.0;
		conn.connected_to = "C1";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		break;
	case 12: 
		res.room = "C1";
		res.x = -1.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		conn.connected_to = "R2";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 13: 
		res.room = "C2";
		res.x = 3.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		conn.connected_to = "C1";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R3";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		conn.connected_to = "R4";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	case 14: 
		res.room = "R1";
		res.x = -7.0;
		res.y = 3.0;
		conn.connected_to = "C1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		break;
	case 15: 
		res.room = "R2";
		res.x = -7.0;
		res.y = -4.0;
		conn.connected_to = "C1";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 16: 
		res.room = "R3";
		res.x = 9.0;
		res.y = 3.0;
		conn.connected_to = "C2";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		break;
	case 17: 
		res.room = "R4";
		res.x = 9.0;
		res.y = -4.0;
		conn.connected_to = "C2";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	default:
		res.room = "no room associated with this marker id";
	}
	return true;
}	

int main(int argc, char **argv)
{
/**
 * \brief The main function for the EXPROBLAB_Assignment2 ROS node.
 *
 * The function initializes the ROS node and creates a service server that listens for requests on the
 * "/room_info" topic. When a request is received, the markerCallback function is called to process the
 * request and provide a response. The function then enters a loop to keep the node running until it is
 * shut down.
 *
 * \param
 * 	argc The number of command line arguments
 * 	argv The command line arguments
 *
 * \return 
 * 	0 on success, non-zero on failure
 */
	ros::init(argc, argv, "EXPROBLAB_Assignment2");
	ros::NodeHandle nh;
	ros::ServiceServer oracle = nh.advertiseService( "/room_info",markerCallback);
	ros::spin();
	ros::shutdown();
	return 0;
}
