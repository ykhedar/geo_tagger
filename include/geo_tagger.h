
#ifndef GEO_TAGGER_H
#define GEO_TAGGER_H

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem/operations.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "sensor_msgs/NavSatFix.h"
#include "geo_tagger/GeoImageCompressed.h"

#include "std_srvs/Trigger.h"

#include <exiv2/exiv2.hpp>

#include <fstream>
#include <cstdlib>

#include <sys/types.h>
#include <pwd.h>

#include <string>

#include "geo_tagger_helper.h"

#include "timer.h"

#define LOGURU_IMPLEMENTATION 1
#define LOGURU_WITH_STREAMS 1
#include "loguru.hpp"

class GeoTagger
{
  public:
    GeoTagger();
  private:
    void gpsCb(const sensor_msgs::NavSatFix::ConstPtr& mGPSmsg)
    {
      mGeoData.mGPS = *mGPSmsg;
      ROS_INFO_STREAM_THROTTLE(10,"Throttled Info.GPS Values in gpsCb()  "<<"Long: "<< mGeoData.mGPS.longitude<< \
      				  " Lat: "<<mGeoData.mGPS.latitude<<" Alt: "<<mGeoData.mGPS.altitude);
      mTimer.reset();
    }
    void imageCb(const sensor_msgs::CompressedImageConstPtr& message);
    bool startGeoSave(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
    bool stopGeoSave(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void arHeadingCb(const std_msgs::Float32ConstPtr& heading_message)
    {
          mGeoData.mCurrentHeading = heading_message->data;
    }
    void arBaroHeightCb(const std_msgs::Float32ConstPtr& baroheight_message)
    {
          mGeoData.mCurrentBaroHeight = baroheight_message->data;
    }

    std::string mCamName, mImgName, mImgSaveLocation, mFileNameStart;

    ExifTagData mExifTagData;
    ImageInfo mImgInfo;

    uint32_t mImgCount;
    bool mSaveStartSrvOn_;
    bool mGPSDummy;
    GeoTagData mGeoData;
    Timer mTimer;
    geo_tagger::GeoImageCompressed mGeoImgROSMsg;
    ros::NodeHandle mNH,mPrivateNH;
    ros::Publisher mGeoImgPublisher;
    ros::Subscriber mImgSubscriber;
    ros::Subscriber mCurrentHeadingSubscriber;
    ros::Subscriber mBaroHeightSubscriber;
    ros::Subscriber mGPSSubscriber;
    ros::Subscriber mMissionStateSubscriber;
    ros::ServiceServer mImgSaveStartService;
    ros::ServiceServer mImgSaveStopService;
};

#endif // GEO_TAGGER_H
