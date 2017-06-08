# pg_driver

## Install steps
* Clone the repo : 
'''
git clone git@github.com:CUHK-PJS/pg_driver.git
'''
* Download the Spinnaker-1.1.0.43(64 bits) SDK from www.ptgrey.com
* unpack the SDK, and follow the installing instruction in the README file
* link the lib file to the pg_driver: 
'''
cd pg_driver
ln -s <path-to-spinnaker-lib> lib
'''
* run catkin_make to install

## Running
'''
roslaunch pg_driver pg_driver_node
'''

## TODO
The only last thing is transform the Spinnaker defined ImagePtr into the opencv cv::Mat, the detail please refer to line 208~212 in pg_driver_node.cpp


