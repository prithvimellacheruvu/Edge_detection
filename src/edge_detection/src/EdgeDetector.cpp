#include <edge_detection/EdgeDetector.hpp>

using namespace edge_detection;
using namespace cv;
using namespace std;

// Your class methods definition goes here

void EdgeDetector::read(String dir_input)
{
    img_input = imread(dir_input);
}

void EdgeDetector::read(sensor_msgs::Image img_inp)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_inp,
                                 sensor_msgs::image_encodings::RGB8);
    img_input = cv_ptr->image;

}

void EdgeDetector::readDepth(sensor_msgs::Image img_inp)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_inp);
    depthImg_input = cv_ptr->image;
}

void EdgeDetector::runDetector()
{
    // Convert to graycsale
    Mat img_gray;
    cvtColor(img_input, img_gray, COLOR_BGR2GRAY);
    // Blur the image for better edge detection
    Mat img_blur;
    GaussianBlur(img_gray, img_blur, Size(3,3), 0);

    // Sobel edge detection
    Mat sobelx, sobely, sobelxy;
    Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
    Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
    Sobel(img_blur, sobelxy, CV_64F, 1, 1, 5);

    // Canny edge detection
    Canny(img_blur, edges, 100, 200, 3, false);
}

void EdgeDetector::drawLines()
{
    Mat dst = edges;

    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    cdstP = cdst.clone();
    // Standard Hough Line Transform
    vector<Vec2f> lines; // will hold the results of the detection
    HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,255,0), 3, LINE_AA);
    }
    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, LINE_AA);
    }

}

Mat EdgeDetector::result()
{
    //    experiment : bitwise_or green lines on the original image - not the best way for sure ;-)
    //    bitwise_or(img_input, cdstP, img_output);

    img_output = img_input + cdstP;
    return img_output;
}

Mat EdgeDetector::edgeResult()
{
    img_output = edges;
    return img_output;
}

void EdgeDetector::displayImage(Mat img_display)
{
    imshow("output", img_display);
    waitKey();
}

sensor_msgs::PointCloud EdgeDetector::get3DLocs(std::vector<cv::Point> edgesVec)
{
    sensor_msgs::PointCloud ptCloud;
    geometry_msgs::Point32 Dpoint;

    auto fx = camInfo_global.P[0];      auto fy = camInfo_global.P[5];
    auto cx = camInfo_global.P[2];      auto cy = camInfo_global.P[6];

    // Camera Perspective Projection Model for a simple pinHole camera model

    for(unsigned long i=0; i < edgesVec.size(); i++)
    {
        Dpoint.x = (edgesVec[i].x - cx) * depthImg_input.at<uchar>(edgesVec[i].x, edgesVec[i].y) / fx;
        Dpoint.y = (edgesVec[i].y - cy) * depthImg_input.at<uchar>(edgesVec[i].x, edgesVec[i].y) / fy;
        Dpoint.z = depthImg_input.at<uchar>(edgesVec[i].x, edgesVec[i].y);
        ptCloud.points.push_back(Dpoint);
    }

    //    x = (u - u0) * depth[v, u] / fx
    //    y = (v - v0) * depth[v, u] / fy
    //    z = depth[v, u]

    //    ptCloud.header.frame_id = "base_camera_1_depth_optical_frame";
    //    The below frame selected from tf_tree
    ptCloud.header.frame_id = "camera_color_optical_frame";
    counter++;
    return ptCloud;
}
