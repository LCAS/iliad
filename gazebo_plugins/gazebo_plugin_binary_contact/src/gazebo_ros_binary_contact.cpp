/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simplified version of Bumper controller
 * by
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * Modifications by MFC
 */

#include <gazebo_plugin_binary_contact/gazebo_ros_binary_contact.h>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBinaryContact)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBinaryContact::GazeboRosBinaryContact() : SensorPlugin()
{
    ROS_ERROR("Binary contact created!!!");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBinaryContact::~GazeboRosBinaryContact()
{
  this->rosnode_->shutdown();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBinaryContact::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_parent);

  if (!this->parentSensor){
      ROS_ERROR("Contact sensor parent is not of type ContactSensor");
  }


  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // "publishing contact/collisions to this topic name:
  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->GetElement("bumperTopicName"))
    this->bumper_topic_name_ =
      _sdf->GetElement("bumperTopicName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);


  //Publish to iot_update
  this->iot_pub_ = this->rosnode_->advertise<diagnostic_msgs::KeyValue>(
    std::string("/collisions"), 1);
  this->update_status_msg_.key = std::string(this->bumper_topic_name_);
  //initial value, not to be send...
  this->update_status_msg_.value.compare("CLOSED");

  /*
  this->contact_pub_ = this->rosnode_->advertise<gazebo_ami::BinaryContact>(
    std::string(this->bumper_topic_name_), 1);
  */


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = this->parentSensor->ConnectUpdated(
     boost::bind(&GazeboRosBinaryContact::OnContact, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
/*
 void GazeboRosBinaryContact::OnContactOLD()
{
  if (this->contact_pub_.getNumSubscribers() <= 0)
    return;

  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();


  this->bin_contact_state_msg_.header.frame_id = this->frame_name_;
  this->bin_contact_state_msg_.header.stamp = ros::Time(contacts.time().sec(),
                               contacts.time().nsec());

 this->bin_contact_state_msg_.hasContact.data = (contacts.contact_size()!=0);

  // GetContacts returns all contacts on the collision body


  this->contact_pub_.publish(this->bin_contact_state_msg_);
}*/

// Update the controller
void GazeboRosBinaryContact::OnContact()
{


  msgs::Contacts contacts;
  // up to gazebo  7
  //contacts = this->parentSensor->GetContacts();
  contacts = this->parentSensor->Contacts();
  // there is a contact!
  if (contacts.contact_size()!=0) {
    // do not resend contacts...
    if  (this->update_status_msg_.value.compare("OPEN")!=0)  {
        //ROS_INFO("Sensor was %s we are puting it into Open",update_status_msg_.value.c_str());
        this->update_status_msg_.value = std::string("OPEN");
        this->iot_pub_.publish(this->update_status_msg_);
    }
  } else {
        if  (this->update_status_msg_.value.compare("CLOSED")!=0)  {
        //ROS_INFO("Sensor was %s we are puting it into Closed",update_status_msg_.value.c_str());
        this->update_status_msg_.value = std::string("CLOSED");
        this->iot_pub_.publish(this->update_status_msg_);
    }
}



}

}
