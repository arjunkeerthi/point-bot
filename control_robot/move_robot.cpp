#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
//#include "opencv2/contrib/contrib.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <cmath>

/* Objects for matchers and publisher */
cv::Ptr<cv::StereoBM> sbm;
cv::Ptr<cv::StereoSGBM> sgbm;
image_transport::Publisher disp_pub;
ros::Publisher depth_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& imageL, const sensor_msgs::ImageConstPtr& imageR) {
    try {

        /* Convert ROS Image messages to OpenCV images, which are then
           converted to grayscale */
        cv::Mat cvImageL = cv_bridge::toCvShare(imageL, "bgr8")->image;
        cv::Mat cvImageR = cv_bridge::toCvShare(imageR, "bgr8")->image;
        cv::imwrite("src/control_robot/left_rectified_image.png", cvImageL);
        cv::imwrite("src/control_robot/right_rectified_image.png", cvImageR);
        cv::Mat grayL, grayR;
        cv::cvtColor(cvImageL, grayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cvImageR, grayR, cv::COLOR_BGR2GRAY);
        cv::Rect rect(108, 60, 216, 120);
        cv::Rect rect2(144, 80, 144, 80);
        cv::rectangle(cvImageL, rect, cv::Scalar(0,0,255));
        cv::rectangle(cvImageL, rect2, cv::Scalar(0,0,255));
        cv::imwrite("src/control_robot/left_rectangle.png", cvImageL);
        /* Display left and right rectified images */
        //cv::imshow("left_rect_color", cvImageL);
        //cv::imshow("right_rect_color", cvImageR);

        /* Run OpenCV's StereoBM block matching algorithm on left and right
           rectified images, then normalize to range 0-255 grayscale (8 bit
           single channel - CV_8U) */
        cv::Mat disp, disp8;
        sbm->compute(grayL, grayR, disp);
        //sgbm->compute(grayL, grayR, disp); /* To run StereoSGBM instead */
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

        /* Perform Gaussian blur on disparity map g_iter times */
        /*int g_iter = 1;
        for(int i = 0; i < g_iter; i++) {
            cv::GaussianBlur(disp8, disp8, cv::Size(5,5), 0);
        }*/


        /* Perform grayscale morphological opening on disparity map */
        /*cv::Mat struct_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
        cv::morphologyEx(disp8, disp8, cv::MORPH_OPEN, struct_element); */

        /* Get total number of pixels in disparity map */
        int total_px = disp8.rows * disp8.cols;


        /* Run either masked zero elimination mean filtering or
           masked zero elimination median filtering
           Reference: https://people.clarkson.edu/~hudsonb/courses/cs611 */
        /*int iter_count = 0;
        while(cv::countNonZero(disp8) < total_px && iter_count < 20) {
        for(int r = 0; r < disp8.rows; r++) {
            for(int c = 0; c < disp8.cols; c++) {
                if(disp8.at<uchar>(r,c) == 0) {
                    //disp<uchar>.at(r,c) = 1;
                    int r1 = r-2 < 0 ? 0 : r-2;
                    int r2 = r+2 >= disp8.rows ? disp8.rows : r+2;
                    int c1 = c-2 < 0 ? 0 : c-2;
                    int c2 = c+2 >= disp8.cols ? disp8.cols : c+2;
                    cv::Mat A = cv::Mat(disp8, cv::Range(r1, r2), cv::Range(c1, c2));
                    int nz = cv::countNonZero(A);
                    if (nz != 0) {
                        // Mean filtering
                        disp8.at<uchar>(r,c) = cv::sum(A)[0] / nz;

                        //Median filtering
                        //std::vector<uchar> v_flat;
                        //for(int ar = 0; ar < A.rows; ar++) {
                        //    for(int ac = 0; ac < A.cols; ac++) {
                        //        if(A.at<uchar>(ar,ac) != 0) {
                        //            v_flat.push_back(A.at<uchar>(ar, ac));
                        //        }
                        //    }
                        //}
                        //std::nth_element(v_flat.begin(), v_flat.begin() + v_flat.size()/2, v_flat.end());
                        //disp8.at<uchar>(r,c) = v_flat[v_flat.size()/2];
                    }
                }
            }
        }
        ++iter_count;
        }*/

        /* Gaussian Blur after mean or median filtering */
        /* int g_iter2 = 1;
        for(int i = 0; i < g_iter2; g++) {
            cv::GaussianBlur(disp8, disp8, cv::Size(5,5), 0);
        }*/

        /* Box filter blur */
        /*for(int i = 0; i < 2; i++) {
            cv::blur(disp8, disp8, cv::Size(5,5));
        }*/

        /* Do grayscale morphological closing on disparity map */
        /*cv::Mat struct_element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
        int morph_close_iter = 1;
        for(int i = 0; i < morph_close_iter; i++) {
            cv::morphologyEx(disp8, disp8, cv::MORPH_CLOSE, struct_element2);
        }*/

        /* Apply jet colormap to disparity before displaying */
        /* cv::applyColorMap(disp8, disp8, cv::COLORMAP_JET);
        cv::imshow("disparity", disp8); */

        cv::imwrite("src/control_robot/disparity_image.png", disp8);

        /* Convert disparity image to ROS Image message and publish to
           "/control_robot/disparity_image" topic */
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", disp8).toImageMsg();
        disp_pub.publish(msg);
        /*for(int r = 0; r < disp8.rows; r++) {
            for(int c = 0; c < disp8.cols; c++) {
                disp8.at<float>(r, c) = 1;
            }
        }*/


        /* TODO: Work on attempts at depth below or move to separate node */

        /* Will only compute depth on middle section (divide disparity map
           into 3x3 grid and compute average depth on middle rectangle) */
        cv::Mat middle = cv::Mat(disp8, cv::Range(80,160), cv::Range(144,288));

        /* Construct Q matrix using camera matrix from camera calibration */
        /*float Q_data[16] = { 1, 0, 0, -214.1453037549374, 0, 1, 0, -122.8193944709537, 0, 0, -1/0.064, (214.1453037549374-223.5827604021137)/0.064 };
        cv::Mat Q = cv::Mat(4, 4, CV_32F, Q_data);*/

        /* Calculate 3D points for each pixel in disparity map.
           Uses (wx,wy,wz,w) = Q * (c, r, disparity, 1); */
        /*middle.convertTo(middle, CV_32F, 1./16);
        middle.convertTo(middle, CV_32F);
        cv::Mat norm_depth = cv::Mat(middle.rows, middle.cols, CV_32F);
        for(int r = 0; r < middle.rows; r++) {
            for(int c = 0; c < middle.cols; c++) {
                uchar d = middle.at<uchar>(r,c);
                float pts[4] = {c, r, d, 1};
                cv::Mat s = cv::Mat(4,1,CV_32F,pts);
                cv::Mat t = Q * s;
                float t3 = t.at<float>(3,1);
                if(abs(t3) < 0.0001) {
                    if(t3 >= 0) {
                        t3 = 0.0001;
                    } else {
                        t3 = -0.0001;
                    }
                }
                float x = t.at<float>(0,1) / t3;
                float y = t.at<float>(1,1) / t3;
                float z = t.at<float>(2,1) / t3;
                // Currently attempting to just calculate distance of 3D point
                // from origin (assumed to be left camera)
                norm_depth.at<float>(r,c) = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
                //norm_depth.at<float>(r,c) = std::abs(z);
            }
        }
        cv::Mat test_depth;
        normalize(norm_depth, test_depth, 0, 255, CV_MINMAX, CV_8U);
        std::cout << test_depth << std::endl << std::endl;*/

        /* Try to use OpenCV's reprojectTo3D() function - currently giving
           many INF value due to holes in disparity map, hence the above manual
           implementation that deals with case where disparity is close to zero.
           Also reasoning for trying to fill holes in disparity map with
           blurring, morphological operations, and mean/median filtering */
        /*cv::Mat disp32;
        disp.convertTo(disp32, CV_32F, 1./16);
        cv::Mat output3d(disp32.size(), CV_32FC3);
        cv::reprojectImageTo3D(disp32, output3d, Q);
        cv::Mat test_depth(output3d.rows, output3d.cols, CV_32F);
        cv::Mat ch1, ch2, ch3;
        std::vector<cv::Mat> channels(3);
        cv::split(output3d, channels);
        ch1 = channels[0];
        ch2 = channels[1];
        ch3 = channels[2];
        std::cout << ch1 << std::endl << std::endl;
        cv::pow(ch1, 2, ch1);
        cv::pow(ch2, 2, ch2);
        cv::pow(ch3, 2, ch3);
        cv::Mat norm_depth = cv::Mat(output3d.rows, output3d.cols, CV_32F);
        cv::add(ch1, ch2, norm_depth);
        cv::add(norm_depth, ch3, norm_depth);
        cv::sqrt(norm_depth, norm_depth);
        normalize(norm_depth, norm_depth, 0, 255, CV_MINMAX, CV_8U);*/
        //std::cout << "M = " << std::endl << " " << norm_depth << std::endl << std::endl;*/

        /* Another attempt at depth calculation, using
           depth = focal_length * baseline / disparity.
           Fix hole problem by increasing values from 0 to 1 (not good either) */
        middle += 1;
        //std::cout << middle << std::endl << std::endl;
        middle.convertTo(middle, CV_32F);
        float fx = 340.3432526334113;
        float fy = 340.53122558882;
        float b = 0.064;
        cv::Mat test_depth = (fx * b) / middle;
        normalize(test_depth, test_depth, 0, 255, CV_MINMAX, CV_8U);

        /* Display depth map */
        //std::cout << test_depth << std::endl << std::endl;
        //cv::applyColorMap(test_depth, test_depth, cv::COLORMAP_JET);
        cv::imshow("middle_depth", test_depth);
        cv::imwrite("middle_depth.png", test_depth);

        cv::Scalar average = cv::mean(test_depth);
        float avg_depth = average[0];
        std::string s("FAR");
        if(avg_depth < 170 && avg_depth > 130) {
            s = "NEAR";
        } else if (avg_depth <= 130) {
            s = "CLOSE!!!";
        }
        std::cout << s << std::endl;

        geometry_msgs::Point d;
        d.x = avg_depth;
        depth_pub.publish(d);

        cv::waitKey(30);
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imageL->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    /* Initialize and start node */
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    /* Create windows for displaying images */
    //cv::namedWindow("left_rect_color");
    //cv::namedWindow("right_rect_color");
    //cv::namedWindow("disparity");
    //cv::namedWindow("depth");
    //cv::startWindowThread();


    /* Initialize parameters for StereoBM block matcher
       Parameters used from: https://jayrambhia.com/blog/disparity-mpas */
    sbm = cv::StereoBM::create(16, 9);
    sbm->setPreFilterCap(61);
    sbm->setPreFilterSize(5);
    sbm->setMinDisparity(-39);
    sbm->setTextureThreshold(507);
    sbm->setUniquenessRatio(0);
    sbm->setSpeckleWindowSize(0);
    sbm->setSpeckleRange(8);
    sbm->setDisp12MaxDiff(1);

    /* Initialize parameters for StereoSGBM matcher.
       Parameters used from: https://jayrambhia.com/blog/disparity-mpas

    minDisparity = -64
    numDisparities = 192
    blockSize = 9
    P1 = 600
    P2 = 2400
    disp12MaxDiff = 10
    preFilterCap = 4
    uniquenessRatio = 1
    speckleWindowSize = 150
    speckleRange = 2
    mode = StereoSGBM::MODE_SGBM
    */
    //sgbm = cv::StereoSGBM::create(-64, 192, 9, 600, 2400, 10, 4, 1, 150, 2);


    /* Initialize image transport for subscribing to left and right rectified images */
    image_transport::ImageTransport it(nh);
    image_transport::SubscriberFilter subLeft(it, "/stereo/left/image_rect_color", 1);
    image_transport::SubscriberFilter subRight(it, "/stereo/right/image_rect_color", 1);

    /* Publisher to publish disparity map */
    disp_pub = it.advertise("control_robot/disparity_image", 1);
    depth_pub = nh.advertise<geometry_msgs::Point>("control_robot/middle_depth", 1);

    /* Approximate Time Synchronizer is used to get images from left and right
       cameras that approximately have the same timestamp */
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subLeft, subRight);

    /* Call imageCallback() on each pair of images recieved */
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));
    ros::spin();

    //cv::destroyAllWindows();
    return 0;
}
