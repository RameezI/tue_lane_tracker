#include <SigInt.h>
#include "FrameFeeder.h"
#include "StateMachine.h"
#include "LaneModel.h"
#include "boost/program_options.hpp"

#include "ros/ros.h"
#include "tue_lane_tracker/Waypoints.h"
#include <ros/xmlrpc_manager.h>
#include <dynamic_reconfigure/server.h>
#include "tue_lane_tracker/reboot.h"
#include "tue_lane_tracker/LaneTrackerConfig.h"


using namespace std;
namespace po = boost::program_options;


//Fucntion definitions
unique_ptr<FrameFeeder> createFrameFeeder(FrameSource srcMode, string srcString);
bool reboot(tue_lane_tracker::reboot::Request  &req, tue_lane_tracker::reboot::Response &res);
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

// Signal-safe flag for whether ros-kill shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
sig_atomic_t volatile g_request_reboot 	 = 0;


//Local Parameter Server
class ParmServer
{
  public:
  LaneTracker::Config mConfig;

  public:
  void callback(tue_lane_tracker::LaneTrackerConfig& Config, uint32_t level);

  LaneTracker::Config& getConfig()
  {
	return mConfig;
  }

  void setConfig(const tue_lane_tracker::LaneTrackerConfig Config)
  {
	mConfig.lane_avg_width 		= Config.lane_avg_width;
        mConfig.lane_std_width 		= Config.lane_std_width;
	mConfig.lane_min_width 		= Config.lane_min_width;
	mConfig.lane_max_width 		= Config.lane_max_width;
	mConfig.lane_marker_width	= Config.lane_marker_width;
          
        //Camera Resolution
        mConfig.cam_res_v		= Config.cam_res_v;
        mConfig.cam_res_h 		= Config.cam_res_h;

        // Camera Intrinsic parameters
        mConfig.cam_fx  		= Config.cam_fx;
        mConfig.cam_fy			= Config.cam_fy;

        mConfig.cam_cx			= Config.cam_cx;
        mConfig.cam_cy			= Config.cam_cy;

        mConfig.cam_pitch		= Config.cam_pitch; 
        mConfig.cam_yaw			= Config.cam_yaw;
        mConfig.cam_height		= Config.cam_height;
        mConfig.cam_lateral_offset	= Config.cam_lateral_offset;

	//LaneFilter VpFilter Paramters 
        mConfig.base_line_IBCS       	= Config.base_line_IBCS;
        mConfig.purview_line_IBCS    	= Config.purview_line_IBCS;      
        mConfig.step_lane_filter_cm  	= Config.step_lane_filter_cm;
        mConfig.step_vp_filter       	= Config.step_vp_filter;
        mConfig.vp_range_ver         	= Config.vp_range_ver; 
        mConfig.vp_range_hor         	= Config.vp_range_hor;
        mConfig.buffer_count         	= Config.buffer_count;
        mConfig.display_graphics     	= Config.display_graphics;
  }

};
 


int main(int argc, char* argv[]) /**
	This is the entry point of the application.
	- Initialises the sigInit handler
	- Creates a stateMachine and spins it until user issues a quit signal through the sigInt handler.
	*/
{
	int lReturn 	= 0;

	FrameSource 	lFrameSource;
	std::string 	lSourceStr;


	// Parsing command line options
	{

	  po::options_description  lDesc("Options");

	  lDesc.add_options()
	  ("help,h",
			"\t produces help message")

	  ("Mode,m",
			po::value<FrameSource>(&lFrameSource)->default_value(FrameSource::DIRECTORY),
			"\t selects frame input mode")

	  ("Source,s",
			po::value<string>(&lSourceStr)->default_value("../DataSet"),
			"\t provides source configuration");

	  po::variables_map vMap;
	  po::store(po::parse_command_line(argc, argv, lDesc), vMap);
	  po::notify(vMap);

	  if ( vMap.count("help") )
	  {
 	     cout << lDesc <<endl;
	     cout << "	Valid arguments for 'Mode': ";
	     cout << "["<<FrameSource::DIRECTORY<<" ";
	     cout <<FrameSource::STREAM<<" ";
	     cout <<FrameSource::GMSL<<"]";

	     cout <<endl<<endl<< "Examples:"<<endl;
	     cout<< "	./TUeLaneTracker -m " << FrameSource::DIRECTORY << " -s " << "/home/DataSet" <<endl;
	     cout<<endl<<endl;
	     lReturn = 1;
	  }

	} // End parsing command line options


	try
	{
	 ros::init(argc, argv, "lane_tracker", ros::init_options::NoSigintHandler);
 	 signal(SIGINT, &SigInt::handler);
	}
	catch(...)
	{
	 cout<<"ros initialisation failed"<<endl;
	 lReturn = -1;
	}

	ros::NodeHandle   ros_handle;
	ParmServer 	  lParmServer;

	ros::ServiceServer service = ros_handle.advertiseService("lane_tracker/reboot", reboot);
	dynamic_reconfigure::Server<tue_lane_tracker::LaneTrackerConfig> parm_server;
	dynamic_reconfigure::Server<tue_lane_tracker::LaneTrackerConfig>::CallbackType cb;
	cb = boost::bind(&ParmServer::callback,&lParmServer, _1, _2);
	parm_server.setCallback(cb);

	ros::Publisher Waypoints_pub = ros_handle.advertise<tue_lane_tracker::Waypoints>("lane_tracker/way_points", 2);




	unique_ptr<FrameFeeder> lPtrFeeder;
	if (lReturn == 0) //create FrameFeeder
	{
	  lPtrFeeder = createFrameFeeder(lFrameSource, lSourceStr);
	  if(lPtrFeeder == nullptr)
	  {
	    lReturn = -1;
	  }
	}


	shared_ptr<SigInt> lPtrSigInt;
	if(lReturn == 0) //create SigInt
	{
	  // Override XMLRPC shutdown
  	  ros::XMLRPCManager::instance()->unbind("shutdown");
  	  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	  lPtrSigInt = make_shared<SigInt>();
	  if(lPtrSigInt->sStatus == SigStatus::FAILURE)
	  {
	    lReturn = -1;
	  }
	}

	LaneTracker::Config& lConfig = lParmServer.getConfig();
	LaneModel lLaneModel;


	unique_ptr<StateMachine> lPtrStateMachine;
	if (lReturn==0) //create StateMachine
	{
	  std::cout<<endl<<endl;
	  std::cout<<"******************************"<<std::endl;
	  std::cout<<" Press Ctrl + c to terminate."<<std::endl;
	  std::cout<<"******************************"<<std::endl;

	  try
	  {
	       lPtrStateMachine.reset( new StateMachine( move(lPtrFeeder), lConfig ) );
	  }
	  catch(const char* msg)
	  {
		cout<<"******************************"<<endl
		<<"Failed to create the StateMachine!"<<endl
		<< msg <<endl
		<<"******************************"<<endl<<endl;
		lReturn =-1;
	  }
	  catch(...)
	  {
		lReturn = -1;
	  }
	}


	States lPreviousState;
	if (lReturn ==0) //Get current State of the stateMachine
	{
	  cout<<lPtrStateMachine->getCurrentState();
	  lPreviousState = lPtrStateMachine->getCurrentState();
	}

	
	if(lReturn == 0) //spin the stateMachine
        {
    	  uint64_t        lCyclesCount = 0;
    	  uint64_t        lSeq = 0;
    	  ProfilerLDT     lProfiler;
    	  StateMachine&   stateMachine = *lPtrStateMachine.get();


	  while (stateMachine.getCurrentState() != States::DISPOSED )
	  {

	    // spin the state-Machine
	    ros::spinOnce();
	    
            if ( (lPtrSigInt->sStatus == SigStatus::STOP) | (g_request_shutdown==1))
	    {
               stateMachine.quit();
	    }

	    if (g_request_reboot==1)
	    {
	       stateMachine.reboot();
	       g_request_reboot = 0;
	    }

	    lProfiler.start("StateMachine_Cycle");

            lReturn = stateMachine.spin();

	   if(stateMachine.laneModel()) //if laneModel exists
	   {
	      lLaneModel = stateMachine.getLaneModel();

	      geometry_msgs::Point lPoint;
	      tue_lane_tracker::Waypoints msg;

	      //Populate header
	      msg.header.seq 		= lSeq;
	      msg.header.stamp		= ros::Time::now();
	      msg.header.frame_id	= "/vehicle_symmetry_plane";

	      lPoint.x = (lLaneModel.boundaryLeft_cm[0] + lLaneModel.boundaryRight_cm[0])/2.0;
	      lPoint.y = lLaneModel.lookAheadPts_cm[0];
	      lPoint.z = 0;
	      msg.points.push_back(lPoint);

	      lPoint.x = (lLaneModel.boundaryLeft_cm[1] + lLaneModel.boundaryRight_cm[1])/2.0;
	      lPoint.y = lLaneModel.lookAheadPts_cm[1]; 
	      lPoint.z = 0;
	      msg.points.push_back(lPoint);

	      Waypoints_pub.publish(msg); //publish Waypoints

	   }

           lCyclesCount ++;
	
	   lProfiler.end();

	   if(lPreviousState != stateMachine.getCurrentState())
	   {
		cout<<endl<<stateMachine.getCurrentState();
		std::cout.flush();
		lPreviousState = stateMachine.getCurrentState();
	   }

	   else if (lCyclesCount%100==0)
	   {
	 	cout <<endl<<stateMachine.getCurrentState();
		cout <<"state cycle-count = " << lCyclesCount<<"    Cycle-Time [Min, Avg, Max] : "
		<<"[ "<<lProfiler.getMinTime("StateMachine_Cycle")<<" "
		<<lProfiler.getAvgTime("StateMachine_Cycle")<<" "
		<<lProfiler.getMaxTime("StateMachine_Cycle")<<" "
		<<" ]";
	   }

	  }// End spinning
	}

      	lPtrStateMachine.reset( nullptr);

       	cout<<endl<<"The node ended with exit code: " <<lReturn<<endl;
       	ros::shutdown();

       	return(lReturn);
}


unique_ptr<FrameFeeder> createFrameFeeder(FrameSource srcMode, string srcString)
{
	unique_ptr<FrameFeeder>	lPtrFeeder;

	/** Create Image Feeder */
	try
	{
	  switch(srcMode)
	  {
            case DIRECTORY:
               lPtrFeeder=  unique_ptr<FrameFeeder>( new ImgStoreFeeder(srcString) );
               break;
            case STREAM:
              lPtrFeeder=  unique_ptr<FrameFeeder>( new StreamFeeder(srcString) );
              break;
            case GMSL:
              throw "NOT IMPLEMENTED";
              break;
	  }
	}
	catch(const char* msg)
	{
	    cout<<"******************************"<<endl
	    <<"Failed to create the FrameFeeder!"<<endl
	    << msg <<endl
	    <<"******************************"<<endl<<endl;
	    lPtrFeeder = nullptr;
	}
	catch (...)
	{
	    cout<<"******************************"<<endl
	    <<"Failed to create the FrameFeeder!"<<endl
	    << "Unknown exception"<<endl
	    <<"******************************"<<endl<<endl;
	   lPtrFeeder = nullptr;
	}
	return lPtrFeeder;

}

void ParmServer::callback(tue_lane_tracker::LaneTrackerConfig& Config, uint32_t level)
{
  cout<<endl<<"New configuration recieved from the dynamic parameter server"<<endl;
  cout<<"Rebooting..."<<endl<<endl;
  setConfig(Config);

  g_request_reboot=1;
}

bool reboot(tue_lane_tracker::reboot::Request  &req, tue_lane_tracker::reboot::Response &res)
{
   cout<<endl<<"Rebooting..."<<endl<<endl;

   res.config_loaded = true;
   g_request_reboot=1;
   return true;
}


// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}
