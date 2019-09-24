/*
 * Copyright 2015 Aldebaran
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

#ifndef TABLET_EVENT_REGISTER_HPP
#define TABLET_EVENT_REGISTER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <qi/session.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "../src/converters/tablet.hpp"
// Publisher
#include "../src/publishers/basic.hpp"
// Recorder
#include "../recorder/basic_event.hpp"

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
template<class T>
class TabletEventRegister: public boost::enable_shared_from_this<TabletEventRegister<T> >
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  TabletEventRegister();
  TabletEventRegister(const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session );
  ~TabletEventRegister();

  void resetPublisher( ros::NodeHandle& nh );
  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr );

  void startProcess();
  void stopProcess();

  void writeDump(const ros::Time& time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

  void tabletCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message);
  void tabletCallbackMessage(std::string &key, qi::AnyValue &value, geometry_msgs::PointStamped &msg);


private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  boost::shared_ptr<converter::TabletEventConverter<T> > converter_;
  boost::shared_ptr<publisher::BasicPublisher<T> > publisher_;
  //boost::shared_ptr<recorder::BasicEventRecorder<T> > recorder_;

  qi::SessionPtr session_;
  qi::AnyObject p_memory_;
  unsigned int serviceId;
  std::string name_;

  boost::mutex mutex_;

  bool isStarted_;
  bool isPublishing_;
  bool isRecording_;
  bool isDumping_;

protected:
  std::vector<std::string> keys_;
}; // class


class TouchDownEventRegister: public TabletEventRegister<geometry_msgs::PointStamped>
{
public:
  TouchDownEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session ) : TabletEventRegister<geometry_msgs::PointStamped>(name, keys, frequency, session) {}
};

//QI_REGISTER_OBJECT(TouchDownEventRegister, tabletCallback)

static bool _qiregisterTabletEventRegisterTouchDown() {
  ::qi::ObjectTypeBuilder<TabletEventRegister<geometry_msgs::PointStamped> > b;
  QI_VAARGS_APPLY(__QI_REGISTER_ELEMENT, TabletEventRegister<geometry_msgs::PointStamped>, tabletCallback)
    b.registerType();
  return true;
  }
static bool BOOST_PP_CAT(__qi_registration, __LINE__) = _qiregisterTabletEventRegisterTouchDown();

} //naoqi

#endif
