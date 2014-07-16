/************************************************************************/
/* ros-qvision                                                      */
/* A ROS package for robot vision using the Qt and OpenCV               */
/*                                                                      */
/* ImageConverter.cpp                                                   */
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

#include "qvision/ImageConverter.h"

ImageConverter::ImageConverter(ros::NodeHandle nh_): QObject()
    , it_(nh_)
  {
    nh_.param<std::string>("/subscribed_image_topic_name", sub_image_name, "/camera/rgb/image_raw");
    image_sub_ = it_.subscribe(sub_image_name, 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    initialized = false;
  }
cv::Mat ImageConverter::getImage()
{
  return image;
}
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     mutex.lock();
      image = cv_ptr->image;
     mutex.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

     image_pub_.publish(cv_ptr->toImageMsg());
     emit this->newImageObtained();
  
    if(!initialized)
        initialized = true;
  }


