#include <edge_detection/EdgeDetector.hpp>

using namespace std;
using namespace cv;

int counter = 0;
bool checkImg = false;
sensor_msgs::Image img_output;
Mat edges_global;
edge_detection::EdgeDetector detector;
void subImage_Callback(sensor_msgs::Image img_input)
{
    detector.read(img_input);
    detector.runDetector();
    detector.drawLines();
    Mat result = detector.result();
    edges_global = detector.edgeResult();

    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, result);
    img_bridge.toImageMsg(img_output); // from cv_bridge to sensor_msgs::Image
    checkImg = true;
}

bool checkCam = false;
void subCamInfo_Callback(sensor_msgs::CameraInfo camInfo)
{
    detector.camInfo_global = camInfo;
    checkCam = true;

}
bool checkDepth = false;
void subDepthImage_Callback(sensor_msgs::Image depthImg_input)
{
    detector.readDepth(depthImg_input);
    checkDepth = true;
}

int main(int argc, char** argv)
{
    //    ROS Initialization
    ros::init(argc, argv, "Robocop");
    // Define node handles
    ros::NodeHandle nh_glob, nh_loc("~") ;
    //     Declare your node's subscriptions and service clients
    ros::Subscriber subImage = nh_glob.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, subImage_Callback) ;
    ros::Subscriber subCamInfo = nh_glob.subscribe<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 1, subCamInfo_Callback) ;
    ros::Subscriber subDepthImage = nh_glob.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1, subDepthImage_Callback) ;

    ros::Publisher pubName = nh_glob.advertise<sensor_msgs::Image>("/detectedEdges", 1) ;
    ros::Publisher pubEdgePoints = nh_glob.advertise<sensor_msgs::PointCloud>("/edge_points", 1) ;

    while (ros::ok()){
        ros::spinOnce();
        counter++;
        if(checkImg)
        {
            pubName.publish(img_output);
            ROS_INFO_ONCE("Publishing to topic /detectedEdges ... ");
        }

        if(checkCam && checkImg)
        {
            ROS_INFO_ONCE("Recieved Camera Info ... ");

            std::vector<cv::Point> edgesVec;
            sensor_msgs::PointCloud ptCloud;
            for (int i=0; i<edges_global.rows; i++)
            {
                for (int j=0; j<edges_global.cols; j++)
                {
                    int pixVal = edges_global.at<uchar>(i,j);
                    if (pixVal > 250)
                    {
                        edgesVec.push_back({i,j});
                    }
                }
            }
            if (checkDepth)
            {
                ROS_INFO_ONCE("Recieved depth Info & calc ptCloud ... ");
                ptCloud = detector.get3DLocs(edgesVec);
                pubEdgePoints.publish(ptCloud);
                ROS_INFO_ONCE("Publishing ptCloud to /edge_points ... ");
            }

        }


    }
}



