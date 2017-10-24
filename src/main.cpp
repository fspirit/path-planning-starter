#include <fstream>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"


#include "KeepLanePathPlanner.h"
#include "SimpleSplineBasedPlanner.h"
#include "WebSocketMessageHandler.h"

using std::string;
using std::cout;
using std::endl;
using std::cerr;

const int StartingLane = 1;


int main(int argc, char * argv[])
{
	uWS::Hub h;
    HighwayMap map("../data/highway_map.csv");
	SimpleSplineBasedPlanner pathPlanner(map, StartingLane);
	WebSocketMessageHandler handler(pathPlanner);

	h.onMessage([&handler](uWS::WebSocket<uWS::SERVER> ws,
						   char * data,
						   size_t length,
						   uWS::OpCode opCode)
				{
					if (length == 0)
						return;

					string message (data, length);
					handler.HandleMessage(message, ws);
				});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
				   {
					   cout << "Connected" << endl;
				   });

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char * message, size_t length)
					  {
						  ws.close();
						  cout << "Disconnected" << endl;
					  });

	const int port = 4567;
	if (h.listen(port))
		cout << "Listening to port " << port << endl;
	else
		cerr << "Failed to listen to port" << endl;

	h.run();
}