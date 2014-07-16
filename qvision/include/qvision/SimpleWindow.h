/************************************************************************/
/* ros-qvision                                                     */
/* A ROS package for robot vision using the Qt and OpenCV               */
/*                                                                      */
/* SimpleWindow.h                                                       */
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

#ifndef SIMPLEWINDOW_H
#define SIMPLEWINDOW_H

#include "ros/ros.h"
#include "ui_SimpleWindow.h"
#include "qvision/ImageConverter.h"
#include "qvision/rosThread.h"

#include "cvblob.h"
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <opencv/cv.h>
#include <map>
#include <string>

#define QUOTE_(x) #x
#define QUOTE(x) QUOTE_(x)
#define VERION 1.0.1


using namespace std;
using namespace cv;
using namespace cvb;

class SimpleWindow : public QMainWindow,  private Ui::SimpleWindow
{
    Q_OBJECT

public:
    SimpleWindow(QWidget *parent = 0);
    ~SimpleWindow();
    ROSThread* rt;
    cv::Mat rgb, frame;
    
    CvTracks tracks;
    QImage img;
    bool enableCamDisp;
    bool enableFaceRec;
    bool enableRedRec;
    std::string fn_haar;
    cv::CascadeClassifier haar_cascade;
    void blob_track(IplImage* input_image, cv::Mat& output_image);

protected:
    void run();

public slots:
    void connectToCamera();
    void connectToFaceTracker();
    void aboutWindow();
    void updateFrame();
    void startFaceDetection();
};

#endif
