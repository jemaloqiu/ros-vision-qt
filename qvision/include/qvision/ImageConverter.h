/************************************************************************/
/* ros-qvision                                                     */
/* A ROS package for robot vision using the Qt and OpenCV               */
/*                                                                      */
/* ImageConverter.h                                                     */
/*                                                                      */
/* Zhaopeng QIU <qiuzhaopeng@gmail.com>                                 */
/*                                                                      */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files (the "Software"), to deal in the Software without restriction, */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/


#ifndef QV_IMAGE_CONVERTER_H
#define QV_IMAGE_CONVERTER_H

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>
#include <map>


// OPENCV Iincludes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include "cvblob.h"
#include <QObject>
#include <QMetaType>
#include <QMutex>



using namespace std;


class ImageConverter:public QObject
{
Q_OBJECT

public:
    std::string sub_image_name;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr;
    IplImage* current_image_, depth_image;
    cv::Mat image;
    QMutex mutex;  

    int counter;
    bool initialized;

signals:
    void newImageObtained();
    void reqFaceDetected();

public:
    ImageConverter(ros::NodeHandle nh_);
    cv::Mat getImage();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};



#endif
