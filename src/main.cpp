#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
//#include "helper.cpp"
#include "defines.h"

#define LC 2 	//lane centre
#define LW 4   //lane width

#define DISTINC 0.5

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



vector<double>  check_currentlane_behaviour(vector<vector<double >> sensor_fusion, int lane, vector<double> params,unsigned int pp_size ){
//vector<double> params = {car_x, car_y, car_s, car_d, car_yaw, car_speed, end_path_s, end_path_d }; total 8
	double own_s;
	bool status = true;
	for (unsigned int k = 0; k < sensor_fusion.size();++k){
		// [ id, x, y, vx, vy, s, d]

		vector<double> nc = sensor_fusion[k];

		if (nc[6] > (LC+LW*lane - 2) && (nc[6] < (LC+LW*lane +2) ) ){   // if its in the same lane

			double nc_speed = sqrt(nc[3] *nc[3] + nc[4]*nc[4]);  // next car's speed
			double nc_next_s = nc[5] + (pp_size +1)*SIM_TICK*nc_speed;   //cars_next location
			 own_s = (pp_size > 0)? params[6]: params[2] ;

			if( ( nc_next_s > own_s) && ((nc_next_s - own_s) <  LANE_HORIZON ))
				status =false;
				break;
		 	 }
	}

	return {status,own_s} ;   // false if car ahead, otherwise true

}

int getlane (double value)
{
	int clane = -1;
	if ((value > 0 ) && (value < LW ))
		clane = 0;
	else if ((value > LW ) && (value < 2*LW ))
		clane = 1;
	else if((value > 2*LW ) && (value < 3*LW ))
		clane = 2;

	return clane;
}





vector<double>  check_fwd_behaviour(vector<vector<double >> sensor_fusion, int lane, vector<double> params,unsigned int pp_size ){
//vector<double> params = {car_x, car_y, car_s, car_d, car_yaw, car_speed, end_path_s, end_path_d }; total 8

	double status = 1;
	int clane = getlane(params[3]); // get actual car lane
	vector<double> id_ahead = {0,0}, dist_ahead= {LRGENUM,LRGENUM};
	double nc_id;
	for (unsigned int k = 0; k < sensor_fusion.size();++k){
		// [ id, x, y, vx, vy, s, d]

		vector<double>  nc = sensor_fusion[k];

		if (nc[6] > (LC+LW*lane - 2) && (nc[6] < (LC+LW*lane +2) ) ){   // if its in the same lane

			double nc_speed = sqrt(nc[3] *nc[3] + nc[4]*nc[4]);  // next car's speed
			double nc_next_s = nc[5] + (pp_size +1)*SIM_TICK*nc_speed;   //cars_next location
			double own_s = (pp_size > 0)? params[6]: params[2] ;

			if( ( nc_next_s > own_s) && ((nc_next_s - own_s) <  LANE_HORIZON )){  // Find the closest car in horizon
				status = 0;
				if (dist_ahead[0] == LRGENUM) { // only first time
					dist_ahead[0] = nc_next_s - own_s;
					id_ahead[0] = nc[0];
					}
				else {
					dist_ahead[1] = nc_next_s - own_s ;
					id_ahead[1] = nc[0];


					if ( MIN(dist_ahead[1],dist_ahead[0]) == dist_ahead[1]){
						dist_ahead[0] = dist_ahead[1];
						id_ahead[0] = id_ahead[1];
						}

				nc_id=k;
				cout<<nc[0]<<nc_id<<sensor_fusion[k][0]<<endl;
				}
		 	 } // end of horizon check
		} // end of loop for cars in same lane
	} // end of for loop for checking all cars

	return {status,id_ahead[0],dist_ahead[0]};   // false if car ahead, otherwise true

}



vector<double>  check_bck_behaviour(vector<vector<double >> sensor_fusion, int lane, vector<double> params,unsigned int pp_size ){
//vector<double> params = {car_x, car_y, car_s, car_d, car_yaw, car_speed, end_path_s, end_path_d }; total 8

	double status =1;
	int clane = getlane(params[3]); // get actual car lane
	vector<double> id_ahead = {0,0}, dist_ahead= {LRGENUM,LRGENUM};

	for (unsigned int k = 0; k < sensor_fusion.size();++k){
		// [ id, x, y, vx, vy, s, d]

		vector<double>  nc = sensor_fusion[k];
		int nc_id;
		if (nc[6] > (LC+LW*lane - 2) && (nc[6] < (LC+LW*lane +2) ) ){   // if its in the same lane

			double nc_speed = sqrt(nc[3] *nc[3] + nc[4]*nc[4]);  // next car's speed
			double nc_next_s = nc[5] + (pp_size +1)*SIM_TICK*nc_speed;   //cars_next location
			double own_s = (pp_size > 0)? params[6]: params[2] ;

			if( ( nc_next_s < own_s) && (( own_s -nc_next_s) <  LANE_HORIZON  )){  // Find the closest car in horizon
				status = 0;
				if (dist_ahead[0] == LRGENUM) { // only first time

					dist_ahead[0] = own_s - nc_next_s;
					cout<<""<<"Chnaged distance first time "<<dist_ahead[0] <<endl;
					id_ahead[0] = nc[0];
					}
				else {
					dist_ahead[1] = own_s - nc_next_s ;
					cout<<""<<"Chnaged distance first time "<<dist_ahead[1] <<endl;
					id_ahead[1] = nc[0];
					if ( MIN(dist_ahead[1],dist_ahead[0]) == dist_ahead[1]){
						dist_ahead[0] = dist_ahead[1];
						id_ahead[0] = id_ahead[1];
						}
					cout<<""<<"Minmi "<<dist_ahead[0] <<endl;
				    }
		 	 } // end of horizon check
		} // end of loop for cars in same lane
	} // end of for loop for checking all cars

	return {status, id_ahead[0],dist_ahead[0]} ;   // false if car ahead, otherwise true

}


double calc_cost (double inp){
	double val = 1 - exp(-1/inp);
	return val;
}

vector<double> check_costs(vector<vector<double>> sensor_fusion, int lane, vector<double> params,unsigned int pp_size ){


	double new_lane = 0, cost1=10000,cost2 = 10000;
 // only support single lane changes and not double from extreme left or right
	if (lane == 0){
		new_lane = lane +1;
		auto fwd_1 = check_fwd_behaviour(sensor_fusion, new_lane,  params, pp_size );
		cost1 =calc_cost(fwd_1[2]);
		cout<<""<<"distance ahead in new right lane path "<<fwd_1[2] <<endl;
		auto bck_1 = check_bck_behaviour(sensor_fusion, new_lane, params, pp_size );
		cost1 +=calc_cost(bck_1[2]);
		cout<<""<<"distance behind the new right  path "<<bck_1[2] <<endl;

	}

	 if (lane == 1){
			auto fwd_1 = check_fwd_behaviour(sensor_fusion, lane + 1,  params, pp_size );
			cost1 =calc_cost(fwd_1[2]);
			cout<<""<<"distance ahead in new right lane path "<<fwd_1[2] <<endl;
			auto bck_1 = check_bck_behaviour(sensor_fusion, lane + 1, params, pp_size );
			cost1 +=calc_cost(bck_1[2]);
			cout<<""<<"distance behind the new right path "<<bck_1[2] <<endl;

			auto fwd_2 = check_fwd_behaviour(sensor_fusion, lane - 1,  params, pp_size );
			cost2 =calc_cost(fwd_2[2]);
			cout<<""<<"distance behind the new left path "<<fwd_2[2] <<endl;
			auto bck_2 = check_bck_behaviour(sensor_fusion, lane - 1, params, pp_size );
			cost2 +=calc_cost(bck_2[2]);
			cout<<""<<"distance behind the new left path "<<bck_2[2] <<endl;
			if (cost1 <= cost2)
				new_lane = lane + 1;
			else {
				new_lane = lane - 1;
				cost1 =cost2;
			     }
	}

	if (lane == 2){
			new_lane = lane - 1;
			auto fwd_1 = check_fwd_behaviour(sensor_fusion, new_lane ,  params, pp_size );
			cost1 =calc_cost(fwd_1[2]);
			cout<<""<<"distance ahead the new left path "<<fwd_1[2] <<endl;
			auto bck_1 = check_bck_behaviour(sensor_fusion, new_lane , params, pp_size );
			cost1 +=calc_cost(bck_1[2]);
			cout<<""<<"distance behind the new left path "<<bck_1[2] <<endl;

		}

return {new_lane,cost1};

}





void global2car( vector<double> &ptx , vector<double> &pty, double x, double y, double yaw)
{
	for (int k = 0; k < ptx.size(); ++k){
		double x_origin = ptx[k] - x;
		double y_origin = pty[k] - y;

		ptx[k] = x_origin * cos(-yaw) - y_origin*sin(-yaw);
		pty[k] = x_origin * sin( -yaw) + y_origin*cos(-yaw);

		}
}


void get_next_pts(vector<double> &x_pts, vector<double>& y_pts, vector<double> & next_x, vector<double> &next_y,\
		tk::spline s, unsigned int pp_size, double x, double y, double yaw, double & velocity)
{

	double x_horizon = PATH_HORIZON ;  // horizon
	double y_horizon = s(x_horizon);
	double path = sqrt((x_horizon*x_horizon)  + (y_horizon*y_horizon));

	double x_addon = 0;


	for(int k = 1; k < PATH_POINTS - pp_size; ++k){

		double N = 2.24* path / (SIM_TICK * velocity);
		double x_point = x_addon + (x_horizon/N);
		double y_point = s(x_point);

		x_addon = x_point ;

		double x_glo_cors = x_point *cos(yaw) - y_point*sin(yaw);
		double y_glo_cors = x_point *sin(yaw) + y_point*cos(yaw);

		x_glo_cors += x;
		y_glo_cors += y;

		next_x.push_back(x_glo_cors);
		next_y.push_back(y_glo_cors);

		}
}





// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  double ref_vel = 0;
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	vector<double> params = {car_x,car_y,car_s,car_d,car_yaw,car_speed,end_path_s, end_path_d };
          	//cout<<" car_d "<<car_d <<" car_s "<<car_s<<" car_speed "<<car_speed<<" end_path_s "<<end_path_s<<endl<<endl;




          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

        	unsigned int pp_size = previous_path_x.size();   //previous path size

        	int lane = getlane(car_d);
        	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	for(int k = 0; k < previous_path_x.size(); ++k){
				next_x_vals.push_back(previous_path_x[k]);
				next_y_vals.push_back(previous_path_y[k]);

			}

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	vector<double> x_pts;
          	vector<double> y_pts;

          	double x_ref = car_x;
			double y_ref = car_y;
			double yaw_ref = deg2rad(car_yaw);



			//prepare_trajectory(x_pts,y_pts, x_ref, y_ref, yaw_ref,map_waypoints_s,map_waypoints_x,map_waypoints_y,pp_size,lane);


			auto result = check_fwd_behaviour(sensor_fusion, lane, params,pp_size);

			if (0 == result[0]){
				auto lane_cost= check_costs(sensor_fusion, lane, params,pp_size);

				auto cost1 = calc_cost(result[2]);
				cout<<""<<"distance in straight path "<<result[2] <<endl;
				cout<< "lane change cost "<< lane_cost[1]<< "staying in lane cost" <<cost1<<endl<<endl;
				if (cost1 > lane_cost[1]){
					cout<<"doing lane change, new"<<lane_cost[0]<< " old "<<lane<< endl<<endl;
					lane = lane_cost[0];
				}
				ref_vel -= 2.0/2.24;

			}
			else if (ref_vel < REF_VEL){
				ref_vel += 2.0/2.24;
					if (ref_vel >  REF_VEL)
					ref_vel = REF_VEL;
				}

//			cout<< "car ahead has is ID" << result[0]<< " distance "<<result[1]<<" ID "<<sensor_fusion[result[2]][0]<<endl;
			//check_currentlane_behaviour(sensor_fusion, lane, params,pp_size );

			/*for (unsigned int k = 0; k < sensor_fusion.size();++k){
				// [ id, x, y, vx, vy, s, d]

				vector<double> nc = sensor_fusion[k];

				if (nc[6] > (LC+LW*lane - 2) && (nc[6] < (LC+LW*lane +2) ) ){   // if its in the same lane

					double nc_speed = sqrt(nc[3] *nc[3] + nc[4]*nc[4]);  // next car's speed
					double nc_next_s = nc[5] + (pp_size +1)*SIM_TICK*nc_speed;   //cars_next location
					double own_s = (pp_size > 0)? end_path_s: car_s ;

					if( ( nc_next_s > own_s) && ((nc_next_s - own_s) <  LANE_HORIZON ))
						too_close = 1;
						break;
				 	 }
			}*/




			// get future trajectory
          	if (pp_size < 2) { // simulation just started
          		x_pts.push_back(car_x - cos(car_yaw));

          		x_pts.push_back(car_x);

          		y_pts.push_back(car_y - sin(car_yaw));

          		y_pts.push_back(car_y);

          	}
          	else {  // car has a previous path

          		x_ref = previous_path_x[pp_size - 1];
          		y_ref = previous_path_y[pp_size - 1];

          		double x_ref_prev = previous_path_x[pp_size - 2];
          		double y_ref_prev = previous_path_y[pp_size - 2];

          		x_pts.push_back(x_ref_prev);
          		x_pts.push_back(x_ref);

          		y_pts.push_back(y_ref_prev);
          		y_pts.push_back(y_ref);

          		yaw_ref = atan2(y_ref - y_ref_prev,x_ref-x_ref_prev);


          	}



          	vector<double> wp0 = getXY(car_s + 40,(LC + LW* lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			vector<double> wp1 = getXY(car_s + 70,(LC + LW* lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			vector<double> wp2 = getXY(car_s + 90,(LC + LW*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);


			x_pts.push_back(wp0[0]);
			x_pts.push_back(wp1[0]);
			x_pts.push_back(wp2[0]);

			y_pts.push_back(wp0[1]);
			y_pts.push_back(wp1[1]);
			y_pts.push_back(wp2[1]);


			// change to car coordinates
			global2car(x_pts , y_pts, x_ref, y_ref, yaw_ref );

			tk::spline s;  //declare spline
			s.set_points(x_pts,y_pts); // spline for 5 points

			get_next_pts(x_pts,y_pts,next_x_vals, next_y_vals, s, pp_size,x_ref,y_ref, yaw_ref,ref_vel);


          	// Finished

          	// Sending points to the simulator
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































