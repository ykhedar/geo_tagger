#include "../include/geo_tagger.h"
#include <algorithm>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geo_tagger_flight_node");
  char log_path[PATH_MAX];
  loguru::suggest_log_path("~/loguru/", log_path, sizeof(log_path));
  loguru::add_file(log_path, loguru::FileMode::Truncate, loguru::Verbosity_MAX);

  GeoTagger geotagger_;
  ros::spin();
  return 0;
}

GeoTagger::GeoTagger(): mImgCount(0), mSaveStartSrvOn_(false), mTimer(std::chrono::seconds(3)), mPrivateNH("~")
{

  //- Get data from parametr server
  std::string flight_save_start_srv, flight_save_stop_srv, mImgTopic, mGeoImgTopic;
  std::string ar_head_topic, ar_baro_topic, gps_topic;
  mPrivateNH.param("image_topic", mImgTopic, std::string("uninitialised"));
  mPrivateNH.param("camera_name", mCamName, std::string("uninitialised"));
  mPrivateNH.param("geoimage_topic", mGeoImgTopic, std::string("uninitialised"));
  mPrivateNH.param("flight_save_start_srv", flight_save_start_srv, std::string("uninitialised"));
  mPrivateNH.param("flight_save_stop_srv", flight_save_stop_srv, std::string("uninitialised"));
  mPrivateNH.param("ar_head_topic", ar_head_topic, std::string("uninitialised"));
  mPrivateNH.param("ar_baro_topic", ar_baro_topic, std::string("uninitialised"));
  mPrivateNH.param("gps_topic", gps_topic, std::string("uninitialised"));
  mPrivateNH.param("gps_dummy", mGPSDummy, false);


  mImgSaveStartService = mNH.advertiseService(flight_save_start_srv, &GeoTagger::startGeoSave, this);
  mImgSaveStopService = mNH.advertiseService(flight_save_stop_srv, &GeoTagger::stopGeoSave, this);

  mImgSubscriber = mNH.subscribe(mImgTopic, 200,&GeoTagger::imageCb, this);
  mCurrentHeadingSubscriber    = mNH.subscribe(ar_head_topic, 200,&GeoTagger::arHeadingCb, this);
  mBaroHeightSubscriber        = mNH.subscribe(ar_baro_topic, 200,&GeoTagger::arBaroHeightCb, this);
  mGPSSubscriber               = mNH.subscribe(gps_topic,200,&GeoTagger::gpsCb, this);
  mGeoImgPublisher = mNH.advertise<custom_msgs::GeoImageCompressed>(mGeoImgTopic,200);
  ROS_INFO_STREAM("\n" << "GeoTagger has been successfully created."  << "\n" \
  				  "Image Topic Subscribed is: 	   " << mImgTopic    	<< "\n" \
  				  "Camera Name is:				   " << mCamName 	    << "\n" \
  				  "GeoImage is Published on topic: " << mGeoImgTopic);

  LOG_SCOPE_F(INFO, "Will indent all log messages within this scope.");
  LOG_S(INFO)<< "\n" << "GeoTagger has been successfully created."  << "\n" \
            "Image Topic Subscribed is:      " << mImgTopic     << "\n" \
            "Camera Name is:           " << mCamName      << "\n" \
            "GeoImage is Published on topic: " << mGeoImgTopic;
  // TODO Initialise the Messages to null values
  if (mGPSDummy)
  {
      ROS_WARN_STREAM("The GPS is in Dummy Mode. This is useful for test purpose only.");
      mImgSaveLocation = getFlightSaveLocation(mFileNameStart, 13579, "ankdata_drone");
      createSaveLocation(mImgSaveLocation);
  }

  if(mCamName == "manta"){
    mFileNameStart = "rgb";
  }
  else if (mCamName == "thermal")
  {
    mFileNameStart = "th";
  }

}

void GeoTagger::imageCb(const sensor_msgs::CompressedImageConstPtr& message)
{
  if (mGPSDummy)
  {
      mGeoData.mGPS.longitude = 10;
      mGeoData.mGPS.latitude = 52;
      mGeoData.mGPS.altitude = 10;
      mTimer.reset();
  }

  
  if (mTimer.isEnded())
  {
  	ROS_WARN_STREAM_THROTTLE(10,"No GPS Fix. No GeoImage will be saved or published.");
    LOG_S(INFO)<<"No GPS Fix. No GeoImage will be saved or published.";
  	return;
  }

  

  if (mSaveStartSrvOn_||mGPSDummy)
  {
    // Publish GeoImage
    ros::Time lCurrentTime = ros::Time::now();
    boost::posix_time::ptime my_posix_time = lCurrentTime.toBoost();
    std::string time_string = boost::posix_time::to_iso_string(my_posix_time);
    std::replace( time_string.begin(), time_string.end(), 'T', '_');

    std::string time_exiftag = boost::posix_time::to_simple_string(my_posix_time);
    mImgInfo.mImgOrigTime = time_exiftag;
    mImgInfo.mCamName = mCamName;
    mImgInfo.mImgName = mImgSaveLocation + "/" + time_string.substr(9, 9) + "_" + mFileNameStart + "_" + std::to_string(mImgCount) + ".jpg";
    mImgCount++;
    mImgInfo.mImgCount = mImgCount;

    mGeoImgROSMsg.imagename.data=mImgInfo.mImgName.substr(mImgSaveLocation.length());
    mGeoImgROSMsg.imagedata = *message;
    mGeoImgROSMsg.gpsdata=mGeoData.mGPS;
    mGeoImgROSMsg.heading.data = mGeoData.mCurrentHeading;
    mGeoImgROSMsg.baroHeight.data = mGeoData.mCurrentBaroHeight;
    mGeoImgROSMsg.header.stamp = lCurrentTime;
    mGeoImgROSMsg.header.seq  = mImgCount;
    mGeoImgROSMsg.missioncount.data = 0;
    mGeoImgPublisher.publish(mGeoImgROSMsg);

    // Save Geotagged Image as JPG
    ExivImagePtr limage = Exiv2::ImageFactory::open(&(message->data)[0], message->data.size());
    geoTagImage(limage,mImgInfo, mGeoData);
  }
}

bool GeoTagger::startGeoSave(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  mSaveStartSrvOn_ = true;
  res.success = mSaveStartSrvOn_;
  ROS_INFO_STREAM("Requested (startGeoSave) and succeded. MissionId is: " << mMissionCount);
  LOG_S(INFO)<<"Requested (startGeoSave) and succeded. MissionId is: " << mMissionCount;
  mImgSaveLocation = getFlightSaveLocation(mFileNameStart, 0, "ankdata_drone");
  createSaveLocation(mImgSaveLocation);
  mImgInfo.mProjectName = mImgSaveLocation.substr(mImgSaveLocation.size()-13);
  mImgCount = 0;
  return true;
}

bool GeoTagger::stopGeoSave(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  mSaveStartSrvOn_ = false;
  res.success = mSaveStartSrvOn_;
  ROS_INFO_STREAM("Requested (stopGeoSave) and succeded.");
  LOG_S(INFO)<<"Requested (stopGeoSave) and succeded.";
  return true;
}
