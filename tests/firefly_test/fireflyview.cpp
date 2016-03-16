#include <FlyCapture2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <sys/time.h>

static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

void printFPS(double &last_tic, int &frame)
{
    frame++;
    if (frame %10 == 0) {
        double t = tic();
        std::cout << "\t" << 10.0 / (t - last_tic) << " fps" << std::endl;
        last_tic = t;
    }
}

int main()
{
    FlyCapture2::Error error;
    FlyCapture2::Camera camera;
    FlyCapture2::CameraInfo camInfo;

    // Connect the camera
    error = camera.Connect(0);
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;
        return false;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;
        return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber << std::endl;

    error = camera.StartCapture();
    if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        return false;
    }
    else if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;
        return false;
    }

    // capture loop
    char key = 0;
    int frame_index = 0;
    double last_tic = tic();
    while(key != 'q')
    {
        // Get the image
        FlyCapture2::Image rawImage;
        FlyCapture2::Error error = camera.RetrieveBuffer( &rawImage );
        if ( error != FlyCapture2::PGRERROR_OK )
        {
                std::cout << "capture error" << std::endl;
                continue;
        }

        // convert to rgb
        FlyCapture2::Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
        cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        cv::imshow("image", image);
        key = cv::waitKey(1);
        printFPS(last_tic, frame_index);
    }

    error = camera.StopCapture();
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    camera.Disconnect();

    return 0;
}
