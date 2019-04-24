#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

boost::shared_ptr<ros::AsyncSpinner> g_spinner;

bool g_enable = false;
bool g_trigger = false;

// called from global callback queue
void enableCb(const std_msgs::BoolConstPtr& msg)
{
  if (g_enable != msg->data)
  {
    g_enable = msg->data;
    g_trigger = true;
  }
}

// called from custom callback queue
void heartbeatCb(const std_msgs::EmptyConstPtr& msg)
{
  ROS_INFO("Heartbeat");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");

  // custom callback queue
  ros::CallbackQueue queue;

  // This node handle uses global callback queue
  ros::NodeHandle n;
  // and this one uses custom queue
  ros::NodeHandle hb_n;
  // Set custom callback queue
  hb_n.setCallbackQueue(&queue);

  ros::Subscriber enable = n.subscribe<std_msgs::Bool>("enable", 10, enableCb);
  // Note that we use different NodeHandle here
  ros::Subscriber hb = hb_n.subscribe<std_msgs::Empty>("heartbeat", 100, heartbeatCb);

  // Create AsyncSpinner, run it on all available cores and make it process custom callback queue
  g_spinner.reset(new ros::AsyncSpinner(0, &queue));

  g_enable = true;
  g_trigger = true;

  // Loop with 100 Hz rate
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    // Enable state changed
    if (g_trigger)
    {
      if (g_enable)
      {
        // Clear old callback from the queue
        queue.clear();
        // Start the spinner
        g_spinner->start();
        ROS_INFO("Spinner enabled");
      }
      else
      {
        // Stop the spinner
        g_spinner->stop();
        ROS_INFO("Spinner disabled");
      }
      // Reset trigger
      g_trigger = false;
    }

    // Process messages on global callback queue
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Release AsyncSpinner object
  g_spinner.reset();

  // Wait for ROS threads to terminate
  ros::waitForShutdown();
}