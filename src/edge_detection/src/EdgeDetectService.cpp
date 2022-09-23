#include <edge_detection/EdgeDetector.hpp>

using namespace std;
using namespace cv;

int counter = 0;
bool check = false;
sensor_msgs::Image input;

bool detectorService(edge_srv::edgeDetector::Request  &request,
                     edge_srv::edgeDetector::Response &response)
{
    ROS_INFO_ONCE("Inside Service ... ");
    //    String dir_input = request.dirInput.dir_input.data;
    edge_detection::EdgeDetector detector;

    //    detector.read(dir_input);
    detector.read(request.input);
    detector.runDetector();
    detector.drawLines();
    Mat result = detector.result();
    //    detector.displayImage(result);
    //    detector.displayImage(detector.edges);

    ROS_INFO_ONCE("Result inside Service ... ");

    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, result);
    img_bridge.toImageMsg(response.output); // from cv_bridge to sensor_msgs::Image

    ROS_INFO_ONCE("Sending response Service ... ");

    return true;
}

void subscriber_Callback(sensor_msgs::Image img_input)
{
    input = img_input;
    check = true;
}

int main(int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "EdgeDetector");
    // Define node handles
    ros::NodeHandle nh_glob, nh_loc("~") ;

    ros::Subscriber subName = nh_glob.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, subscriber_Callback) ;
    ros::Publisher pubName = nh_glob.advertise<sensor_msgs::Image>("/ServiceDetectedEdges", 1) ;
    ros::ServiceServer servi = nh_glob.advertiseService("/edgeDetector", detectorService);
    ros::ServiceClient client = nh_glob.serviceClient<edge_srv::edgeDetector>("edge_srv/edgeDetector");

    while (ros::ok())
    {
        ros::spinOnce();counter++;

        //         create the message and call the service.

        edge_srv::edgeDetector service;
        //        service.request.input = "/home/ecn/Neura_Robotics/src/edge_detection/data/Image_1.png" ;

        cv_bridge::CvImagePtr cv_ptr;
        if(check)
        {   ROS_INFO_ONCE("Image successfully received ... ");
            service.request.input = input;
            ROS_INFO_ONCE("Calling Service ... ");
            if(client.call(service))
            {
                ROS_INFO_ONCE("Service successfully called ... ");
                pubName.publish(service.response.output);


                //                cv_ptr = cv_bridge::toCvCopy(service.response.output,
                //                                             sensor_msgs::image_encodings::RGB8);
                //                Mat result = cv_ptr->image;

            }
            else
                ROS_INFO_ONCE("Service call failed ... ");

        }
    }
    return 0;

}
