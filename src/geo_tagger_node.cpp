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

  mExifTagData = get_camera_config(mPrivateNH);

  mImgSaveStartService = mNH.advertiseService(flight_save_start_srv, &GeoTagger::startGeoSave, this);
  mImgSaveStopService = mNH.advertiseService(flight_save_stop_srv, &GeoTagger::stopGeoSave, this);

  mImgSubscriber = mNH.subscribe(mImgTopic, 200,&GeoTagger::imageCb, this);
  mCurrentHeadingSubscriber    = mNH.subscribe(ar_head_topic, 200,&GeoTagger::arHeadingCb, this);
  mBaroHeightSubscriber        = mNH.subscribe(ar_baro_topic, 200,&GeoTagger::arBaroHeightCb, this);
  mGPSSubscriber               = mNH.subscribe(gps_topic,200,&GeoTagger::gpsCb, this);
  mGeoImgPublisher = mNH.advertise<geo_tagger::GeoImageCompressed>(mGeoImgTopic,200);
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
      mImgSaveLocation = getFlightSaveLocation(mFileNameStart, 13579, "ros_images");
      createSaveLocation(mImgSaveLocation);
  }

  mFileNameStart = "rgb";

}

void GeoTagger::imageCb(const sensor_msgs::CompressedImageConstPtr& message)
{
  if (mGPSDummy)
  {
      mGeoData.mGPS.longitude = 5;
      mGeoData.mGPS.latitude = 5;
      mGeoData.mGPS.altitude = 5;
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
    geoTagImage(limage, mImgInfo, mGeoData, mExifTagData);
  }
}

bool GeoTagger::startGeoSave(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  mSaveStartSrvOn_ = true;
  res.success = mSaveStartSrvOn_;
  mImgSaveLocation = getFlightSaveLocation(mFileNameStart, 0, "ros_images");
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

ExifTagData GeoTagger::get_camera_config(ros::NodeHandle nh)
{
    ExifTagData lExifTagData;

    // Get the camera config from ros parameter server.
    std::string model, make, res_unit, x_res, y_res, focal_plane_x_res, focal_plane_y_res, focal_plane_res_unit, focal;
    int cx, cy, fx, fy;
    nh.param("Model", model, std::string("uninitialised"));
    nh.param("Make", make, std::string("uninitialised"));
    nh.param("ResolutionUnit", res_unit, std::string("uninitialised"));
    nh.param("XResolution", x_res, std::string("uninitialised"));
    nh.param("YResolution", y_res, std::string("uninitialised"));
    nh.param("FocalPlaneXResolution", focal_plane_x_res, std::string("uninitialised"));
    nh.param("FocalPlaneYResolution", focal_plane_y_res, std::string("uninitialised"));
    nh.param("FocalPlaneResolutionUnit", focal_plane_res_unit, std::string("uninitialised"));
    nh.param("FocalLength", focal, std::string("uninitialised"));
    nh.param("cx", cx, 0);
    nh.param("cy", cy, 0);
    nh.param("fx", fx, 0);
    nh.param("fy", fy, 0);
                    
    // Populate he lExifTagData.
    lExifTagData.mExifData["Exif.Image.Model"]    = model;            // Ascii
    lExifTagData.mExifData["Exif.Image.Make"]     = make;          // Ascii
    lExifTagData.mExifData["Exif.Image.ResolutionUnit"]   = res_unit;              // Short, 2 for inches
    lExifTagData.mExifData["Exif.Image.XResolution"]    = x_res;         // Rational
    lExifTagData.mExifData["Exif.Image.YResolution"]    = y_res;         // Rational
    lExifTagData.mExifData["Exif.Photo.FocalPlaneXResolution"]    = focal_plane_x_res; // Rational
    lExifTagData.mExifData["Exif.Photo.FocalPlaneYResolution"]    = focal_plane_y_res; // Rational
    lExifTagData.mExifData["Exif.Photo.FocalPlaneResolutionUnit"]    = focal_plane_res_unit;   // short, 3 for cm
    lExifTagData.mExifData["Exif.Photo.FocalLength"]    = focal;           // Rational
    lExifTagData.mXmpData["Xmp.exif.cx"] = cx;
    lExifTagData.mXmpData["Xmp.exif.cy"] = cy;
    lExifTagData.mXmpData["Xmp.exif.fx"] = fx;
    lExifTagData.mXmpData["Xmp.exif.fy"] = fy;
    
    return lExifTagData;
}