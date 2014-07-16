/************************************************************************/
/* ros-qvision                                                          */
/* A ROS package for robot vision using the Qt and OpenCV               */
/*                                                                      */
/* rosThread.cpp                                                        */
/*                                                                      */
/* Zhaopeng QIU <qiuzhaopeng@gmail.com>                                 */
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

#include "qvision/rosThread.h"

#include <QDebug>

ROSThread::ROSThread()
{  
    int argc = 0;
    char** argv = NULL; 
    ros::init (argc, argv, "qvision"); 
    ros::NodeHandle node; 
    ros::start();
    ic = new ImageConverter(node);

}  

ROSThread::~ROSThread()
{
  ros::shutdown();
  ros::waitForShutdown();
  delete ic;
}


void ROSThread::run()
{
    ROS_INFO("Starting ROS thread");
    ros::spin();

    qDebug() << "Start ROS thread...";
}
