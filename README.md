# pg_driver

## Install steps
* Clone the repo : 
```bash
git clone git@github.com:CUHK-PJS/pg_driver.git
```
* Download the Spinnaker-1.1.0.43(64 bits) SDK from www.ptgrey.com
* unpack the SDK, and follow the installing instruction in the README file
* link the lib file to the pg_driver: 
```bash
cd pg_driver
ln -s <path-to-spinnaker-lib> lib
```
* run catkin_make to install
```bash
roscd
cd ..
catkin_make
```

## Running

* Use rosrun method
```bash
roscore 
rosrun pg_driver pg_driver_node
```

* Use Launch method
```bash
roslaunch pg_driver camera.launch 
```

## View the video
```bash
rosrun image_view image_view image=:/camera/image
```

## Code description
The code is orginized as three module in the main function:
* Camera Setup
This module will check all the free cameras in the current system, and initial them, all this part is copy from Acquisition/Acquisition.cpp file.

* Core module
This is the major parts for extracting the image from driver and publish to the ros space.

Use the following method to extract new image from a single camera.
```cpp
		// Acquire images
		ImagePtr pResultImage = pCam->GetNextImage();
```

The following code aims to transform the driver specified images firest into the opencv format (cv::Mat), then use cv_bridge to transform into ros message.
```cpp
            ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);

            // Convert to CV mat
            unsigned int XPadding = convertedImage->GetXPadding();
            unsigned int YPadding = convertedImage->GetYPadding();
            unsigned int rowsize = convertedImage->GetWidth();
            unsigned int colsize = convertedImage->GetHeight();

            cv::Mat image = cv::Mat(colsize + YPadding, rowsize + XPadding,
                                    CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
```

To publish the message
```cpp
        pub.publish(msg);
```

> Note here, all the publisher handle is defined at the beginning of main function
```cpp
    ros::init(argc, argv, "driver");
    ros::NodeHandle nh;
    ros::Rate loop(1);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
```


* Camera Release
This module will release all the in use cameras, all this part is copy from Acquisition/Acquisition.cpp file.
