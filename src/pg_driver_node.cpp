#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;


// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap)
{
    int result = 0;

    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            FeatureList_t::const_iterator it;
            for (it = features.begin(); it != features.end(); ++it)
            {
                CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = (CValuePtr)pfeatureNode;
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "driver");
    ros::NodeHandle nh;
    ros::Rate loop(1);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE *tempFile = fopen("test.txt", "w+");
    if (tempFile == NULL)
    {
        cout << "Failed to create file in current folder.  Please check "
            "permissions."
            << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    // Print application build information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    //
    // Create shared pointer to camera
    //
    // *** NOTES ***
    // The CameraPtr object is a shared pointer, and will generally clean itself
    // up upon exiting its scope. However, if a shared pointer is created in the
    // same scope that a system object is explicitly released (i.e. this scope),
    // the reference to the shared point must be broken manually.
    //
    // *** LATER ***
    // Shared pointers can be terminated manually by assigning them to NULL.
    // This keeps releasing the system from throwing an exception.
    //

    CameraPtr pCam = NULL;

    // TODO: As current there is only one cam, and let's assume the camera ID is 0
    pCam = camList.GetByIndex(0);

    // Retrieve TL device nodemap and print device information
    INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

    PrintDeviceInfo(nodeMapTLDevice);

    // Initialize camera
    pCam->Init();

    // Retrieve GenICam nodemap
    INodeMap & nodeMap = pCam->GetNodeMap();

    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
        return -1;
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
        return -1;
    }

    // Retrieve integer value from entry node
    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
    cout << "Acquisition mode set to continuous..." << endl;

    // Image acquisition must be ended when no more images are needed.
    pCam->BeginAcquisition();
    cout << "Acquiring images..." << endl;

    gcstring deviceSerialNumber("");
    CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
    if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
    {
        deviceSerialNumber = ptrStringSerial->GetValue();

        cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
    }
    cout << endl;

    //======================================Core Module======================================//
    //======================================Core Module======================================//
    //======================================Core Module======================================//
    int imageCnt = 0;
    sensor_msgs::ImagePtr msg (new sensor_msgs::Image);
    // Retrieve, and publish images
    while(ros::ok()){
        std::cout << "Enter the loop" << std::endl;

        // Acquire images
        ImagePtr pResultImage = pCam->GetNextImage();

        //
        // Ensure image completion
        //
        // *** NOTES ***
        // Images can easily be checked for completion. This should be
        // done whenever a complete image is expected or required.
        // Further, check image status for a little more insight into
        // why an image is incomplete.
        //
        if (pResultImage->IsIncomplete())
        {

            cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
        }
        else
        {
            // Print image information; height and width recorded in pixels
            //
            // *** NOTES ***
            // Images have quite a bit of available metadata including
            // things such as CRC, image status, and offset values, to
            // name a few.
            //
            size_t width = pResultImage->GetWidth();

            size_t height = pResultImage->GetHeight();

            cout << "Grabbed image " << imageCnt++ << ", width = " << width << ", height = " << height << endl;

            //
            // Convert image to mono 8
            //
            // *** NOTES ***
            // Images can be converted between pixel formats by using
            // the appropriate enumeration value. Unlike the original
            // image, the converted one does not need to be released as
            // it does not affect the camera buffer.
            //
            // When converting images, color processing algorithm is an
            // optional parameter.
            //
            ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);

            // Create a unique filename
            ostringstream filename;

            filename << "Acquisition-";
            if (deviceSerialNumber != "")
            {
                    filename << deviceSerialNumber.c_str() << "-";
            }
            filename << imageCnt << ".jpg";

            //
            // Save image
            //
            // *** NOTES ***
            // The standard practice of the examples is to use device
            // serial numbers to keep images of one device from
            // overwriting those of another.
            //
            convertedImage->Save(filename.str().c_str());
            cout << "Image saved at " << filename.str() << endl;

            // Convert to CV mat
            unsigned int XPadding = convertedImage->GetXPadding();
            unsigned int YPadding = convertedImage->GetYPadding();
            unsigned int rowsize = convertedImage->GetWidth();
            unsigned int colsize = convertedImage->GetHeight();

            cv::Mat image = cv::Mat(colsize + YPadding, rowsize + XPadding,
                                    CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        }

        //
        // Release image
        //
        // *** NOTES ***
        // Images retrieved directly from the camera (i.e. non-converted
        // images) need to be released in order to keep from filling the
        // buffer.
        //
        pResultImage->Release();

        pub.publish(msg);
        std::cout << "Image retrieve done" << std::endl;
        ros::spinOnce();
        loop.sleep();
    }

    //======================================================================================//
    //======================================Core Module======================================//
    //======================================Core Module======================================//

    //
    // End acquisition
    //
    // *** NOTES ***
    // Ending acquisition appropriately helps ensure that devices clean up
    // properly and do not need to be power-cycled to maintain integrity.
    //
    pCam->EndAcquisition();

    // Deinitialize camera
    pCam->DeInit();

    //
    // Release reference to the camera
    //
    // *** NOTES ***
    // Had the CameraPtr object been created within the for-loop, it would not
    // be necessary to manually break the reference because the shared pointer
    // would have automatically cleaned itself up upon exiting the loop.
    //
    pCam = NULL;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();

    return 0;
}

