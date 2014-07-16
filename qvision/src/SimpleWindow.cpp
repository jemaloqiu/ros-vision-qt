/************************************************************************/
/* ros-qvision                                                          */
/* A ROS package for robot vision using the Qt and OpenCV               */
/*                                                                      */
/* SimpleWindow.cpp                                                     */
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

#include "qvision/SimpleWindow.h"
#include "qvision/ImageConverter.h"
#include <iostream>
#include <QDebug>
#include <ros/package.h> //for ros::package::getPath 


SimpleWindow::SimpleWindow(QWidget *parent) : QMainWindow(parent)
{  
    setupUi(this);  

    enableCamDisp = false;
    enableFaceRec = true;
    enableRedRec = false;
    std::string res_path = ros::package::getPath("qvision");
    res_path += "/res/";
    fn_haar = res_path + "haarcascades/haarcascade_frontalface_default.xml";
    haar_cascade.load(fn_haar);
    
    connect(ConnectButton, SIGNAL(released()), this, SLOT(connectToCamera()));
    connect(FaceButton, SIGNAL(released()), this, SLOT(connectToFaceTracker()));
    connect(actionAbout, SIGNAL(triggered()), this, SLOT(aboutWindow()));
    rt = new ROSThread();
    rt->start();
    connect(this->rt->ic, SIGNAL(newImageObtained()), this, SLOT( updateFrame() ));
    // connect(this->rt->ic, SIGNAL(reqFaceDetected()), this, SLOT( startFaceDetection()));
  
}

SimpleWindow::~SimpleWindow() 
{
    delete rt;    
} 

void SimpleWindow::connectToCamera()
{
     enableCamDisp =  (!enableCamDisp);
} 


void SimpleWindow::connectToFaceTracker()
{
     enableFaceRec =  (!enableFaceRec);
     enableRedRec =  (!enableFaceRec);
} 

void SimpleWindow::aboutWindow()
{
    QString ver = QUOTE(VERION); 
    QMessageBox::information(this,"About qvision",QString("A ROS-QT-based robot vision package.\n\n Written by Zhaopeng QIU (qiuzhaopeng@gmail.com) \n\nVersion: ")+ver);
} 

void SimpleWindow::startFaceDetection()
{
  //enableFaceRec = true;
}
void SimpleWindow::updateFrame()
{   
    if (enableCamDisp){

         frame = rt->ic->image; 
         cv::Mat original;  
         IplImage* current_image = new IplImage( frame );

        if (enableFaceRec) {
             cvtColor(frame, original, CV_BGR2RGB);
             cv::Mat gray;
             cvtColor(original, gray, CV_BGR2GRAY);

             vector< Rect_<int> > faces;
             cv::Size minSize=Size(100,100);
             cv::Size maxSize=Size(250,250);
             haar_cascade.detectMultiScale(gray, faces,1.1,3,0, minSize, maxSize);
             

            for(int i = 0; i < faces.size(); i++) {
                Rect face_i = faces[i];
                Mat face = gray(face_i);
                rectangle(original, face_i, CV_RGB(0, 255,0), 1);
                string box_text = format("Face %d", i);
                int pos_x = std::max(face_i.tl().x - 10, 0);
                int pos_y = std::max(face_i.tl().y - 10, 0);
                putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
            }
            faces.clear();
          
         }  //if (enableFaceRec)

        if (enableRedRec)
        { 
           // original = frame;
            blob_track(current_image, rgb);
            cvtColor(rgb, original, CV_BGR2RGB);
            
          }   // if (enableRedRec)
         delete current_image;
         img = QImage((const unsigned char*)(original.data), original.cols, original.rows, QImage::Format_RGB888); 
         frameLabel->setPixmap(QPixmap::fromImage(img));
    }  // if (enableCamDisp)
    else
    { frameLabel->setText("No camera frames received.");}
} 



void SimpleWindow::blob_track(IplImage* input_image, cv::Mat& output_image){

      CvSize imgSize = cvGetSize(input_image);

      IplImage *frame = cvCreateImage(imgSize, input_image->depth, input_image->nChannels);
      IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);

      unsigned int blobNumber = 0;
      bool quit = false;

      cvConvertScale(input_image, frame, 1, 0);

      IplImage *segmentated = cvCreateImage(imgSize, 8, 1);

      for (unsigned int j=0; j<imgSize.height; j++)
        for (unsigned int i=0; i<imgSize.width; i++)
        {
          CvScalar c = cvGet2D(frame, j, i);

          double b = ((double)c.val[0])/255.;
          double g = ((double)c.val[1])/255.;
          double r = ((double)c.val[2])/255.;
          unsigned char f = 255*((r>0.2+g)&&(r>0.2+b));

          cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
        }

      cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN, 1);
      IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

      CvBlobs blobs;
      unsigned int result = cvLabel(segmentated, labelImg, blobs);
      cvFilterByArea(blobs, 400, 1000000);
      cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
      cvUpdateTracks(blobs, tracks, 200., 5);
      cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

      cv::Mat img(frame, 0);
      output_image = img;
      
      cvReleaseImage(&labelImg);
      cvReleaseImage(&segmentated);
      cvReleaseImage(&frame);
      cvReleaseBlobs(blobs);
}


