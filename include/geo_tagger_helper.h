#ifndef GEO_TAGGER_HELPER_H
#define GEO_TAGGER_HELPER_H

typedef std::unique_ptr< Exiv2::Image > ExivImagePtr;

struct GeoTagData
{
    sensor_msgs::NavSatFix mGPS;
    float mCurrentHeading;
    float mCurrentBaroHeight;
};

struct ImageInfo
{
  std::string mCamName;
  std::string mImgName;
  uint32_t mImgCount;
  std::string mImgOrigTime;
  std::string mProjectName;
};

struct ExifTagData
{
  Exiv2::ExifData mExifData;
  Exiv2::XmpData mXmpData;
};
//- Declarations
std::string decimalToDegMinSec(double angle);
std::string dateTime();
std::vector<std::string> rosGpsToExivString(const sensor_msgs::NavSatFix& gps_);
std::string getFlightSaveLocation(const std::string camera_name_, int mission_id_);
int createSaveLocation(const std::string save_location);
ExifTagData createExifData(ImageInfo aImgInfo, GeoTagData aGeoData);
void geoTagImage(const ExivImagePtr& image, ImageInfo aImgInfo, GeoTagData aGeoData);

//- Definitions
std::string decimalToDegMinSec(double angle)
{
    std::stringstream result;
    double angleAbs = fabs(angle);
    int angleDeg = floor(angleAbs);
    double angleRem = ( angleAbs - angleDeg )*60;
    int angleMin = floor(angleRem);
    int angleSec = floor( (angleRem - angleMin)*6000 );
    result << angleDeg << "/1 " << angleMin << "/1 " << angleSec << "/100";
    return result.str();
}

std::string dateTime()
{
  time_t     now = time(0);
  struct tm  tstruct;
  char       tim[15];
  tstruct = *localtime(&now);
  strftime(tim, sizeof(tim), "%y%m%d_%H%M%S", &tstruct);
  return tim;
}

std::vector<std::string> rosGpsToExivString(const sensor_msgs::NavSatFix& gps_)
{
 std::vector<std::string> vect(7);
 // GPS Version ID
 vect[0] = "2 2 2 2";
// Altitude
if ( gps_.altitude >= 0.0 ) vect[1] = "0";      // Above Sea Level
else vect[1] = "1";
vect[2] =  std::to_string((int) floor(fabs(gps_.altitude))) + "/1";

// Latitude
if ( gps_.latitude >= 0.0 ) vect[3] = "N";  // Above Equator
else vect[3] = "S";
vect[4] = decimalToDegMinSec(gps_.latitude);
// Longitude
if ( gps_.longitude >= 0.0 ) vect[5] = "E";     // East of green meridian
else vect[5] = "W";
vect[6] = decimalToDegMinSec(gps_.longitude);
return vect;
}

std::string getFlightSaveLocation(const std::string camera_name_, int mission_id_, const std::string base_name_)
{
	std::stringstream fileLocation;
	char *homedir = getpwuid(getuid())->pw_dir;
	fileLocation << homedir << "/" << base_name_ << "/" << dateTime() << "_f" << mission_id_<< "_raw_"  << camera_name_;
  ROS_INFO("Images will be saved in: %s",fileLocation.str().c_str());

	return fileLocation.str();
}
int createSaveLocation(const std::string save_location)
{
	boost::filesystem::path location(save_location);
	if(!boost::filesystem::exists(location))
	{
	  if ( boost::filesystem::create_directories(location) )  // Read,write and execute permissions to only current User
	  {
	    ROS_INFO_STREAM("Created" << save_location << " for saving images." );
      	return 0;
	  }
	  else
	  {
	    ROS_WARN_STREAM("Could not create" << save_location << "for saving images." );
      	return -1;
	  }
	}
	else
	{
	  ROS_WARN_STREAM("Save Directory already exist. Saving in that directory.");
      return 1;
	}
}

ExifTagData createExifData(ImageInfo aImgInfo, GeoTagData aGeoData, ExifTagData aExifTagData)
{
  aExifTagData.mExifData["Exif.Image.ProcessingSoftware"] = "Exiv2 with ROS";   // Ascii
  // General Camera Info //

  std::vector<std::string> vect = rosGpsToExivString(aGeoData.mGPS);
  // General Tags
  aExifTagData.mExifData["Exif.Photo.DateTimeOriginal"]  = aImgInfo.mImgOrigTime;         // Ascii
  aExifTagData.mExifData["Exif.Image.DateTimeOriginal"]  = aImgInfo.mImgOrigTime;         // Ascii
  aExifTagData.mExifData["Exif.Image.DateTime"]  = aImgInfo.mImgOrigTime;                 // Ascii
  aExifTagData.mExifData["Exif.Image.ImageNumber"]       = aImgInfo.mImgCount;            // Long

  // GPS Tag Info //
  aExifTagData.mExifData["Exif.GPSInfo.GPSVersionID"]      = vect[0];            // Byte
  aExifTagData.mExifData["Exif.GPSInfo.GPSLatitudeRef"]    = vect[3];            // Ascii
  aExifTagData.mExifData["Exif.GPSInfo.GPSLatitude"]       = vect[4];            // Rational
  aExifTagData.mExifData["Exif.GPSInfo.GPSLongitudeRef"]   = vect[5];            // Ascii
  aExifTagData.mExifData["Exif.GPSInfo.GPSLongitude"]      = vect[6];            // Rational
  aExifTagData.mExifData["Exif.GPSInfo.GPSAltitudeRef"]    = vect[1];            // Byte
  aExifTagData.mExifData["Exif.GPSInfo.GPSAltitude"]       = vect[2];            // Rational
  aExifTagData.mXmpData["Xmp.exif.Heading"] = aGeoData.mCurrentHeading;
  aExifTagData.mXmpData["Xmp.exif.RelativeAltitude"] = aGeoData.mCurrentBaroHeight;
  aExifTagData.mXmpData["Xmp.exif.ProjectName"] = aImgInfo.mProjectName;

  ROS_INFO_STREAM_THROTTLE(10,"Throttled Info.GPS Values in createExifData  "<<"Long: "<< vect[6]<< \
            " Lat: "<<vect[4]<<" Alt: "<<vect[2]);

  return aExifTagData;
}

void geoTagImage(const ExivImagePtr& image, ImageInfo aImgInfo, GeoTagData aGeoData, ExifTagData bExifTagData)
{
  ExifTagData aExifTagData = createExifData(aImgInfo, aGeoData, bExifTagData);
  if (image.get() != 0)
  {
    image->setExifData(aExifTagData.mExifData);
    image->setXmpData(aExifTagData.mXmpData);
    image->writeMetadata();

    // NOTE: "fs" is empty file which needs to be created, in order for Exiv2 to write the image
    // with Exif Tags. On the fly writing is not supported from the Exiv2 at the moment.
    std::ofstream fs;
    fs.open(aImgInfo.mImgName,std::ios::out);
    fs.close();

    Exiv2::FileIo file(aImgInfo.mImgName);
    file.open();
    file.write(image->io());
    ROS_INFO_STREAM_THROTTLE(10,"Geotagged Image= "<<aImgInfo.mImgName);
  }
  else
  {
    ROS_ERROR("Could not get valid Image in GeoTagger::geoTagImage()");
    return;
  }
}

#endif // GEO_TAGGER_HELPER_H
