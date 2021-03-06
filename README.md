# geo_tagger
A ros driver package developed at Institute für Flugführung, Technische Universität 
Braunschweig for the for the ANKOMMEN project. Geo-tagging of images using Exiv2-0.25 library. 

### geo_tagger_node
Subscribes to the camera images, saves them with some name and then geo-tags them. It publishes 
the Image with GPS info in an `geo_tagger/GeoImageCompressed` msg.
#### Published topics
`/geoimage` Publishes the `geo_tagger/GeoImageCompressed`.
#### Subscribed topics
`~/image/compressed` `sensor_msgs/CompressedImage` Gets the saved image_name from ROS_INFO stream of image_saver node
`/gps/fix` of type `sensor_msgs/NavSatFix` Gets the current GPS information from some GPS node publishing on this topic
#### Running the node
```
roslaunch geo_tagger geo_tagger.launch
```

## Supported hardware
This driver is designed for Allied Vision Manta G917 But it should work
with other cameras as well.
In order to support a new camera, create a copy of the ```config\avt_manta.yaml``` and update the 
exif parameters suited to the new camera. Then replace the name of the config file in the ```launch\geo_tagger.launch```
file.