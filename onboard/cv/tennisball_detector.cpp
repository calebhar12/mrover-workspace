#include "perception.hpp"
#include <cmath>

using namespace std;
using namespace cv;

#ifdef PERCEPTION_DEBUG
struct mouse_data {
  Mat * hsv;
  Mat * depth;
};
static mouse_data m_data;
static Mat hsv;
/* For debug use: print the HSV values at mouseclick locations */
void onMouse(int event, int x, int y, int flags, void* param)
{
  if (event != EVENT_LBUTTONDOWN) return;
  
  //char text[100];
  mouse_data * m_d =  (mouse_data *) param;
  float d = m_d->depth->at<float>(y,x);
  Vec3b p = m_d->hsv->at<Vec3b>(y,x);
  
  printf("Get mouse click at (%d, %d), HSV value is H: %d, S: %d, V:%d, depth is %.2f meters \n", y, x,
	 p.val[0], p.val[1], p.val[2], d);

  //sprintf(text, "Depth=%.2f meters at (%d,%d)", p, y, x);

  //putText(img2, text, Point(10,20), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(0,255,0));
}
#endif



Mat greenFilter(const Mat& src){
    assert(src.type() == CV_8UC3);

    Mat greenOnly;
    // 36 170 80
    Scalar lowerb = Scalar(36, 170, 80);
    Scalar upperb = Scalar(43, 226, 196);
    inRange(src, lowerb, upperb, greenOnly);

    return greenOnly;
}

//###########  David's Code ##########


Mat greenFilter_David(const Mat& src){
    assert(src.type() == CV_8UC3);
    Mat greenOnly;
    //Scalar lowerb = Scalar(26, 50, 30); 
    //Scalar lowerb = Scalar(30, 50, 30);
    //Scalar lowerb = Scalar(30, 50, 80);
    Scalar lowerb = Scalar(30, 50, 30);
    //Scalar upperb = Scalar(64, 210, 255);
    Scalar upperb = Scalar(64, 210, 280);

    inRange(src, lowerb, upperb, greenOnly);
    return greenOnly;
}

bool valid_circle(vector<Point> &contour, float radius){
    //return true;
    float min_radius = 5;
    if(radius<min_radius){
        return false;
    }
    //return true;
    float cnt_area = contourArea(contour);
    float cir_area = radius * radius * M_PI;
    float cnt_peri = arcLength(contour,true);
    float cir_peri = radius * 2 * M_PI;
    if(cnt_peri > cir_peri) return false;
   
    float percentage_area = abs((cnt_area-cir_area)/cir_area);
    float percentage_peri = abs((cnt_peri-cir_peri)/cir_peri);
    float threshold_area = 0.65;
    float threshold_peri = 0.3;
    if (percentage_area<threshold_area && percentage_peri<threshold_peri){
	cout<<"New Circle"<<endl;
   	cout<<"radius: "<< radius<<endl;
    	cout<<"area  : "<< cnt_area<<endl;
    	cout<<"area E: "<< percentage_area<<endl;
    	cout<<"peri E: "<< percentage_peri<<endl;
        cout<<"pass"<<endl;
        return true;
    }else{
        cout<<"fail"<<endl;
        return false;
    }
    
}

float diff(vector<Point> &contour, float radius){
    float cnt_area = contourArea(contour);
    float cir_area = radius * radius * M_PI;
    float percentage = abs((cnt_area-cir_area)/cir_area);
    return percentage;

}

vector<Point2f> findTennisBall_David(Mat &src){
    Scalar color = Scalar(255, 0, 255);
    
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    Mat mask = greenFilter_David(hsv);
    erode(mask, mask, 2);
    dilate(mask, mask, 2);
    imshow("mask_David",mask);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( unsigned i = 0; i < contours.size(); i++ ){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

    

    //unsigned max_index = 0;
    //unsigned max_area = 0;
    vector<Point2f> output;

    
    if(contours.size()==0){
    	return output;
    }

    
    vector<unsigned> indices;
    for(unsigned i = 0; i<contours.size(); i++){
         if(valid_circle(contours_poly[i],radius[i])){
 	    indices.push_back(i);
         }
    }
    if(indices.size()==0){
         return output;
    }

    unsigned best_index = indices[0];
    float best_diff = diff(contours_poly[best_index], radius[best_index]);
    for(auto i: indices){
        float new_diff =  diff(contours_poly[i], radius[i]);
	if(new_diff<best_diff){
           best_index = i;
           best_diff = new_diff;
        }
    }
    //Scalar black = Scalar(0,0,0);
    //for(auto i:indices){ 
    //  circle( src, center[i], (int)radius[i], black, 2, 8, 0 );
    //    
    //}

    output.push_back(center[best_index]);
    circle( src, center[best_index], (int)radius[best_index], color, 2, 8, 0 );
   

    
    return output;



  
}
//####################################


vector<Point2f> findTennisBall(Mat &src, Mat & depth_src){
  
    #ifndef PERCEPTION_DEBUG
    Mat hsv;
    #endif

    cvtColor(src, hsv, COLOR_BGR2HSV);

    Mat mask = greenFilter(hsv);

    #ifdef PERCEPTION_DEBUG
    mouse_data * m_d = & m_data;
    m_d->hsv = &hsv;
    m_d->depth = &depth_src;
    //imshow("hsv", hsv);
    setMouseCallback("image", onMouse, (void *)m_d);    
    //imshow("mask", mask);
    #endif

    // smoothing
    //medianBlur(mask, mask, 11);
    Size ksize(5,5);
    GaussianBlur(mask, mask, ksize, 1, 1, BORDER_DEFAULT );

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( unsigned i = 0; i < contours.size(); i++ ){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

    
    #ifdef PERCEPTION_DEBUG
    /// Draw polygonal contour + bonding rects + circles
    //Mat drawing = Mat::zeros( mask.size(), CV_8UC3);
    for( unsigned i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar(0, 0, 255);
        drawContours( src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        circle( src, center[i], (int)radius[i], color, 2, 8, 0 );
    }
    #endif

    return center;
}

vector<Point2f> findTennisBallCaleb(Mat &src){
  
    #ifndef PERCEPTION_DEBUG
    Mat hsv;
    #endif

    cvtColor(src, hsv, COLOR_BGR2HSV);

    Mat mask = greenFilter(hsv);

    #ifdef PERCEPTION_DEBUG
    mouse_data * m_d = & m_data;
    m_d->hsv = &hsv;
    //imshow("hsv", hsv);
    setMouseCallback("image", onMouse, (void *)m_d);    
    //imshow("mask", mask);
    #endif

    // smoothing
    //medianBlur(mask, mask, 11);
    Size ksize(5,5);
    GaussianBlur(mask, mask, ksize, 1, 1, BORDER_DEFAULT );

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( unsigned i = 0; i < contours.size(); i++ ){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

    
    #ifdef PERCEPTION_DEBUG
    /// Draw polygonal contour + bonding rects + circles
    //Mat drawing = Mat::zeros( mask.size(), CV_8UC3);
    for( unsigned i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar(0, 0, 255);
        drawContours( src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        circle( src, center[i], (int)radius[i], color, 2, 8, 0 );
    }
    #endif

    return center;
}