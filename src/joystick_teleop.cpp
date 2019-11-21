//////////////////////////////////////////////////////////
// ROSfalcon Simple Joystick Controller.
//
// Mayank Patel
// 21/11/2019

#include <iostream>
#include <string>
#include <cmath>
#include <pluginlib/class_loader.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <ros/common.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
//#include <std_srvs/Empty.h>

#include <boost/scoped_ptr.hpp>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"

#include <ipa_manipulation_msgs/PlanToAction.h>
#include <ipa_manipulation_msgs/PlanToGoal.h>
#include <ipa_manipulation_msgs/MoveToAction.h>
#include <ipa_manipulation_msgs/MoveToGoal.h>

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;
FalconDevice m_falconDevice;

/**********************************************
This function initialises the novint falcon controller

NoFalcons is the index of the falcon which you wish to initialise
Index 0 is first falcon.
**********************************************/
 
bool init_falcon(int NoFalcon) 

{
    cout << "Setting up LibUSB" << endl;
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware
    m_falconDevice.setFalconGrip<FalconGripFourButton>(); //Set Grip
    if(!m_falconDevice.open(NoFalcon)) //Open falcon @ index
    {
        cout << "Failed to find falcon" << endl;
        return false;
    }
    else
    {
        cout << "Falcon Found" << endl;
    }

    //There's only one kind of firmware right now, so automatically set that.
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>();
    //Next load the firmware to the device

    bool skip_checksum = false;
    //See if we have firmware
    bool firmware_loaded = false;
    firmware_loaded = m_falconDevice.isFirmwareLoaded();
    if(!firmware_loaded)
    {
        cout << "Loading firmware" << endl;
        uint8_t* firmware_block;
        long firmware_size;
        {

            firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
            firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


            for(int i = 0; i < 20; ++i)
            {
                if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

                {
                    cout << "Firmware loading try failed" <<endl;
                }
                else
                {
                    firmware_loaded = true;
                    break;
                }
            }
        }
    }
    else if(!firmware_loaded)
    {
        cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << endl;
        return false;
    }
    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue" << endl;
        return false;
    }
    cout << "Firmware loaded" << endl;

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set" << endl;
    std::array<int, 3> forces;
    //m_falconDevice.getFalconFirmware()->setForces(forces);
    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    bool homing_reset = false;
    usleep(100000);
    int tryLoad = 0;
    while(!stop) //&& tryLoad < 100)
    {
        if(!m_falconDevice.runIOLoop()) continue;
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in." << endl;

            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
            cout << "Falcon homed." << endl;
            homing_reset = true;
            stop = true;
        }
        tryLoad++;
    }
    /*if(tryLoad >= 100)
    {
        return false;
    }*/

    m_falconDevice.runIOLoop();
    return true;
}

bool plan_trajectory(const std::string& plan_action_topic_name, const std::string& base_frame_id, const std::array<double, 3>& Pos)
{

    actionlib::SimpleActionClient<ipa_manipulation_msgs::PlanToAction> plan_action_client_(plan_action_topic_name, true);
    if (!plan_action_client_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR_NAMED("main","%s action server is not available, please start 'ipa_mnaipulation' node", plan_action_topic_name.c_str());
      return false;
    }

    // conversion from array to geometry_msgs,
    // where the orientation and frame id is fixed
    geometry_msgs::PoseStamped falcon_eef_stamped;
    falcon_eef_stamped.header.frame_id = base_frame_id;
    falcon_eef_stamped.header.stamp = ros::Time::now();
    falcon_eef_stamped.pose.position.x = Pos[0];
    falcon_eef_stamped.pose.position.y = Pos[1];
    falcon_eef_stamped.pose.position.z = Pos[2];
    falcon_eef_stamped.pose.orientation.w = 1.0;
    falcon_eef_stamped.pose.orientation.x = 0.0;
    falcon_eef_stamped.pose.orientation.y = 0.0;
    falcon_eef_stamped.pose.orientation.z = 0.0;

    ipa_manipulation_msgs::PlanToGoal plan_goal;
    plan_goal.expected_poseStamped = falcon_eef_stamped;
    plan_action_client_.sendGoal(plan_goal);

    bool finished_before_timeout = plan_action_client_.waitForResult(ros::Duration(30.0));
    ipa_manipulation_msgs::PlanToResultConstPtr plan_result = plan_action_client_.getResult();
    actionlib::SimpleClientGoalState plan_state = plan_action_client_.getState();
    if (finished_before_timeout == true && plan_result->got_plan == true && plan_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM_NAMED("main","plan Successed with goal pose: \n " << falcon_eef_stamped);
      return true;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("main","plan failed with goal pose: \n " << falcon_eef_stamped);
      return false;
    }

}

bool move_trajectory(const std::string& move_action_topic_name, const std::string& base_frame_id, const std::array<double, 3>& Pos)
{
  actionlib::SimpleActionClient<ipa_manipulation_msgs::MoveToAction> move_action_client_(move_action_topic_name, true);
  if (!move_action_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR_NAMED("main","%s action server is not available, please start 'ipa_mnaipulation' node", move_action_topic_name.c_str());
    return false;
  }

  // falcon give only cartesian position
  // basic orientation and frame id for falcon
  geometry_msgs::PoseStamped falcon_eef_stamped;
  falcon_eef_stamped.header.frame_id = base_frame_id;
  falcon_eef_stamped.header.stamp = ros::Time::now();
  falcon_eef_stamped.pose.position.x = Pos[0];
  falcon_eef_stamped.pose.position.y = Pos[1];
  falcon_eef_stamped.pose.position.z = Pos[2];
  falcon_eef_stamped.pose.orientation.w = 1.0;
  falcon_eef_stamped.pose.orientation.x = 0.0;
  falcon_eef_stamped.pose.orientation.y = 0.0;
  falcon_eef_stamped.pose.orientation.z = 0.0;

  ipa_manipulation_msgs::MoveToGoal move_goal;
  move_goal.target_pos = falcon_eef_stamped;
  move_goal.followed_from_executed_trajectory = false;
  move_action_client_.sendGoal(move_goal);

  bool finished_before_timeout = move_action_client_.waitForResult(ros::Duration(30.0));
  ipa_manipulation_msgs::MoveToResultConstPtr move_result = move_action_client_.getResult();
  actionlib::SimpleClientGoalState move_state = move_action_client_.getState();
  if (finished_before_timeout == true && move_result->arrived == true && move_state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM_NAMED("main","move Successed with goal pose: \n " << falcon_eef_stamped);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("main","move failed with goal pose: \n " << falcon_eef_stamped);
    return false;
  }


}

int main(int argc, char* argv[])
{
    ros::init(argc,argv, "FalconJoystick");
    
    ros::NodeHandle node;
    int falcon_int;
    bool debug;
    //Button 4 is mimic-ing clutch.
    bool clutchPressed, coagPressed;
    std::string base_frame_id, plan_action_topic_name, move_action_topic_name;
    node.param<int>("falcon_number", falcon_int, 0);
    node.param<bool>("falcon_debug", debug, false);
    node.param<bool>("falcon_clutch", clutchPressed, true);
    node.param<bool>("falcon_coag", coagPressed, true);
    node.param<string>("base_frame", base_frame_id, "arm_left_7_link");
    node.param<string>("plan_action_topic_name", plan_action_topic_name, "arm_planning_node/PlanInCartesian");
    node.param<string>("move_action_topic_name", move_action_topic_name, "arm_planning_node/MoveInCartesian");

    bool planActionExist = true;
    bool moveActionExist = true;

    // this will update goal state as start state
    //ros::Publisher goal_state_publisher = node.advertise<std_srvs::Empty>("rviz/moveit/update_goal_state", 1000);

    if(init_falcon(falcon_int))
    {
        cout << "Falcon Initialised Starting ROS Node" << endl;

        m_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();

        //Start ROS Publisher
        ros::Publisher pub = node.advertise<sensor_msgs::Joy>("/falcon/joystick",10);
        ros::Rate loop_rate(1000);

        while(node.ok())
        {
            sensor_msgs::Joy Joystick;
            std::array<double, 3> prevPos;

            Joystick.buttons.resize(1);
            Joystick.axes.resize(3);

            std::array<double,3> forces;
            //Request the current encoder positions:
            std::array<double, 3> Pos;
            std::array<double, 3> newHome, prevHome;
            int buttons;

            if(m_falconDevice.runIOLoop())
            {
                /////////////////////////////////////////////
                Pos = m_falconDevice.getPosition();  //Read in cartesian position

                buttons = m_falconDevice.getFalconGrip()->getDigitalInputs(); //Read in buttons

                //Publish ROS values
                Joystick.buttons[0] = buttons;
                Joystick.axes[0] = Pos[0];
                Joystick.axes[1] = Pos[1];
                Joystick.axes[2] = Pos[2];
                pub.publish(Joystick);
                
                //TODO if Joystick can subscribe to twist message use those forces instead for haptic feedback
                //if
                float KpGainX = -200;
                float KpGainY = -200;
                float KpGainZ = -200;

                float KdGainX = -500;
                float KdGainY = -500;
                float KdGainZ = -500;
                

                // Check if button 4 is pressed, set the forces equal to 0.
                if(buttons == 4 || buttons == 2 || buttons == 1 || buttons == 8){
                    if(buttons == 4 && coagPressed == false){
                        ROS_INFO("Coag Pressed (Button 4)");
                        coagPressed = true;
                    }
                    else if(buttons == 2 && clutchPressed == false){
                        ROS_INFO("Clutch Pressed (Button 2)");
                        clutchPressed = true;
                    }
                    else if(buttons == 1 && planActionExist == false){
                        ROS_INFO("Plan Trajectory Pressed (Button 1)");
                        planActionExist = true;
                    }
                    else if(buttons == 8 && moveActionExist == false){
                        ROS_INFO("Move Trajectory Pressed (Button 8)");
                        moveActionExist = true;
                    }
                    forces[0] = 0;
                    forces[1] = 0;
                    forces[2] = 0;
                }
                else{
                    if(coagPressed == true){
                        ROS_INFO("Coag Released (Button 4)");
                        coagPressed = false;
                        newHome = Pos;
                        ROS_DEBUG_STREAM("x: " << newHome[0] << " y: " << newHome[1] << " z: " << newHome[2]);
                    }
                    else if(clutchPressed == true){
                        ROS_INFO("Clutch Released (Button 2)");
                        clutchPressed = false;
                        newHome = Pos;
                    }
                    else if(planActionExist == true){
                      ROS_INFO("Plan Trajectory Released (Button 1)");
                      planActionExist = false;
                      newHome = Pos;
                      //plan_trajectory(plan_action_topic_name, base_frame_id, Pos);
                    }
                    else if(moveActionExist == true){
                      ROS_INFO("Move Trajectory Released (Button 8)");
                      moveActionExist = false;
                      newHome = Pos;
                      //move_trajectory(move_action_topic_name, base_frame_id, Pos);
                    }
                    //Simple PD controller
                    forces[0] = ((Pos[0] - newHome[0]) * KpGainX) + (Pos[0] - prevPos[0])*KdGainX;
                    forces[1] = ((Pos[1] - newHome[1]) * KpGainY) + (Pos[1] - prevPos[1])*KdGainY;
                    forces[2] = ((Pos[2] - newHome[2]) * KpGainZ) + (Pos[2] - prevPos[2])*KdGainZ;
                }
                m_falconDevice.setForce(forces); //Write falcon forces to driver (Forces updated on IO loop) Should run @ ~1kHz

                if(debug)
                {
                    cout << "Position= " << Pos[0] <<" " << Pos[1] << " " << Pos[2] <<  endl;
                    cout << "newHome  = " << newHome[0] <<" " << newHome[1] << " " << newHome[2] <<  endl;
                    cout << "Error   =" << Pos[0] - newHome[0] <<" " << Pos[1]-newHome[1] << " " << Pos[2] -newHome[2] <<  endl;
                    //cout << "Force= " << forces[0] <<" " << forces[1] << " " << forces[2] <<  endl;
                }
                prevPos = Pos;
                prevHome = newHome;
            }
            loop_rate.sleep();
        }
        m_falconDevice.close();
    }
    return 0;
}
