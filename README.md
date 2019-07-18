#MOST IMPORTANT
1. Changes with regard to the launch file must only be made in the config/config.yaml 
file to avoid problems in launching of nodes.
2. Run only using launch files either ground node or flight node.
# geo_tagger
A ros driver package developed at Institute für Flugführung, Technische Universität 
Braunschweig for the for the ANKOMMEN project. Geo-tagging of images using Exiv2-0.25 library. 
There are two nodes in the package `geo_tagger_flight_node` and `geo_tagger_ground_node`
### geo_tagger_flight_node
Subscribes to the camera images, saves them with a name and then geo-tags them. It publishes 
the Image with GPS info in an `custom_msgs/GeoImage`.
#### Published topics
`/geoimage` Publishes the `custom_msgs/GeoImage`.
#### Subscribed topics
`~/image` Gets the saved image_name from ROS_INFO stream of image_saver node
`/gps/fix` Gets the current GPS information from some GPS node publishing on this topic
#### Running the node
```
roslaunch geo_tagger geo_tagger_flight.launch
```
### geo_tagger_ground_node
Subscribes to the geoimages published by the flight node and saves them with the info from flight node. 
#### Subscribed topics
`~/geoimage` See config.yaml file for exact topic name
#### Running the node
```
roslaunch geo_tagger geo_tagger_flight.launch
```
## Supported hardware
This driver is designed for Allied Vision Manta G917 and Flir A65X thermal cameras. But it should work
with other cameras as well. 



TODO::
1. Read ExifData from xml file
2. Add Reconfigure Server functionality
3. 