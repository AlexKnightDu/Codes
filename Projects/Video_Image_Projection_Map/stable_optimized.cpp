//
//  stable.cpp
//  SLAM_VO_ORB
//
//  Created by ALXD on 10/25/18.
//  Copyright © 2018 ALXD. All rights reserved.
//  The codes are extended from the sample codes of SLAM book of GaoBo

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <string>

using namespace std;
using namespace cv;

bool good_match_file_out = false;
bool match_file_out = false;


string dir_path = "./test/";
string frame_path = "Frames/";
int distance(KeyPoint k, Point p){
    return sqrt(pow((k.pt.x - p.x), 2) + pow((k.pt.y - p.y), 2));
}

bool match_compare(DMatch m1, DMatch m2){
    return m1.distance < m2.distance;
}


Mat frame (string video_filename, vector<Point2f> points, Mat& overlay, Mat& mask, int target_frame, int base_frame = 0)
{
//    int center_x = 0;
//    int center_y = 0;
//    for (auto i = points.begin(); i != points.end(); i++) {
//        center_x += (*i).x;
//        center_y += (*i).y;
//    }

//    Point center(center_x / points.size(), center_y / points.size());
    
    
    Mat img_1 = imread ( dir_path + frame_path + video_filename + "_Frame_" + to_string(base_frame) + ".jpg", CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( dir_path + frame_path + video_filename + "_Frame_" + to_string(base_frame + target_frame) +".jpg", CV_LOAD_IMAGE_COLOR );
    
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );
    
    
//    vector<KeyPoint> near_keypoints_1;
//    for (auto i = keypoints_1.begin(); i != keypoints_1.end(); i++) {
//        if (distance(*i, center) < 1600) {
//            near_keypoints_1.push_back(*i);
//        }
//    }
    
    
    
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
//    descriptor->compute ( img_1, near_keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );
    
    Mat outimg1;
 
//    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//    drawKeypoints( img_1, near_keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//    imshow("ORB特征点",outimg1);
//    waitKey(0);
    
   
    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );
    
    
    
    double min_dist=10000, max_dist=0;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    

    
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
//        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }
    
    
    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );
    cout << "Good match: " << to_string(good_matches.size()) << endl;
//        Mat img_four_match;
    
//    vector< DMatch > four_matches;
//    sort(good_matches.begin(), good_matches.end(), match_compare);
//    auto pos = good_matches.begin();
//    four_matches.push_back(good_matches[0]);
//    for (int i = 1; i < 4; i++) {
////        four_matches.push_back(good_matches[i]);
//        for (; pos != good_matches.end(); pos++) {
//            if (abs(near_keypoints_1[(*pos).queryIdx].pt.x - near_keypoints_1[four_matches[i-1].queryIdx].pt.x) > 100) {
//                if (abs(near_keypoints_1[(*pos).queryIdx].pt.y - near_keypoints_1[four_matches[i-1].queryIdx].pt.y) > 50){
//                    four_matches.push_back(*pos);
//                    break;
//                }
//            }
//        }
//    }
//    int min_interval = 40;
//    pos++;
////    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, four_matches, img_four_match);
////    imshow ( "Choosen", img_four_match );
////    waitKey(0);
//    for (; pos != good_matches.end(); pos++) {
//        if (abs(near_keypoints_1[(*pos).queryIdx].pt.x - near_keypoints_1[four_matches[0].queryIdx].pt.x) > min_interval) {
//            if (abs(near_keypoints_1[(*pos).queryIdx].pt.y - near_keypoints_1[four_matches[0].queryIdx].pt.y < min_interval)){
//                four_matches.push_back(*pos);
//                break;
//            }
//        }
//    }
//    pos++;
//    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, four_matches, img_four_match);
//    imshow ( "Choosen", img_four_match );
//    waitKey(0);
//    for (; pos != good_matches.end(); pos++) {
//        if (abs(near_keypoints_1[(*pos).queryIdx].pt.x - near_keypoints_1[four_matches[1].queryIdx].pt.x) < min_interval) {
//            if (abs(near_keypoints_1[(*pos).queryIdx].pt.y - near_keypoints_1[four_matches[0].queryIdx].pt.y > min_interval)){
//                four_matches.push_back(*pos);
//                break;
//            }
//        }
//    }
//    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, four_matches, img_four_match);
//    imshow ( "Choosen", img_four_match );
//    waitKey(0);
//    
//    pos++;
//    for (; pos != good_matches.end(); pos++) {
//        if (abs(near_keypoints_1[(*pos).queryIdx].pt.x - near_keypoints_1[four_matches[0].queryIdx].pt.x) < min_interval) {
//            if (abs(near_keypoints_1[(*pos).queryIdx].pt.y - near_keypoints_1[four_matches[2].queryIdx].pt.y < min_interval)){
//                four_matches.push_back(*pos);
//                break;
//            }
//        }
//    }
//
//    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, four_matches, img_four_match);
//    imshow ( "Choosen", img_four_match );
//    waitKey(0);
//    for (int i = 0; ; i++) {
//        cout << good_matches[i].distance << endl;
//
//        four_matches.push_back(good_matches[i]);
//    }
    
    Mat img_match;
    Mat img_goodmatch;
    Mat img_four_match;
//    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
//    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, matches, img_match );
//    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
//    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, four_matches, img_four_match);
////    drawMatches ( img_1, near_keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );

//    CvPoint2D32f origin_keypoints[4], transformed_keypoints[4];
//    CvPoint2D32f origin_img_points[4], transformed_img_points[4];
    
    vector<Point2f> origin_keypoints, transformed_keypoints;
    
    
    for (auto i = good_matches.begin(); i != good_matches.end(); i++) {
//    for (auto i = four_matches.begin(); i != four_matches.end(); i++) {
//        origin_keypoints.push_back(Point2f(near_keypoints_1[(*i).queryIdx].pt));
        origin_keypoints.push_back(Point2f(keypoints_1[(*i).queryIdx].pt));
        transformed_keypoints.push_back(Point2f(keypoints_2[(*i).trainIdx].pt));
//        origin_keypoints[i - four_matches.begin()].x = near_keypoints_1[(*i).queryIdx].pt.x;
//        origin_keypoints[i - four_matches.begin()].y = near_keypoints_1[(*i).queryIdx].pt.y;
//        origin_keypoints[i - four_matches.begin()].x = near_keypoints_1[(*i).trainIdx].pt.x;
//        origin_keypoints[i - four_matches.begin()].y = near_keypoints_1[(*i).trainIdx].pt.y;
    }
    
//    for (int i = 0; i < 4; i++) {
//        origin_img_points[i].x = points[i].x;
//        origin_img_points[i].y = points[i].y;
//    }

//    for (int i = 0; i < 4; i++) {
//        cout << origin_keypoints[i].x << ',' << origin_keypoints[i].y << " -- " << transformed_keypoints[i].x << ',' << transformed_keypoints[i].y << endl;
//    }

    Mat warp_matrix = Mat(3,3,CV_32FC1);
    Mat transformed_overlay, transformed_mask;
    vector<Point2f> transformed_points;
    warp_matrix = findHomography(origin_keypoints, transformed_keypoints, CV_RANSAC);
    
//    warp_matrix = findHomography(origin_keypoints, transformed_keypoints);
    warpPerspective(overlay, transformed_overlay, warp_matrix, img_2.size());
    warpPerspective(mask, transformed_mask, warp_matrix, img_2.size());
    
//    cout << transformed_mask << endl;
//    warpPerspective(points, transformed_points, warp_matrix, img_2.size());
//    cvWarpPerspective(&overlay, &transformed_overlay, warp_matrix);
    
    
//    Mat result = img_2 - img_2.mul(transformed_mask) + transformed_overlay.mul(transformed_mask);
    transformed_overlay.copyTo(img_2, transformed_mask);
//    Mat result;
//    addWeighted(img_2, 0, transformed_overlay, 1, 0, result);
    
//
//    drawKeypoints( img_1, four_keypoints, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    
//    imshow("ORB特征点",outimg1);

    
//    imshow ( "All match", img_match );
//    imshow ( "Optimized", img_goodmatch );
//    imshow ( "Choosen", img_four_match );
//    imshow("Overlay", overlay);
//    imshow("Transformed overlay", transformed_overlay);
//    imshow("Overlay result",result);

    if (match_file_out)
        imwrite(dir_path + "match_" + video_filename + "_Frame_" + to_string(base_frame) + "_" + to_string(base_frame + target_frame) + ".jpg", img_match);
    if (good_match_file_out)
        imwrite(dir_path + "good_match_" + video_filename + "_Frame_" + to_string(base_frame) + "_" + to_string(base_frame + target_frame) + ".jpg", img_goodmatch);
//    imwrite(dir_path + "four_match_" + video_filename + "_Frame_" + to_string(base_frame) + "_" + to_string(base_frame + target_frame) + ".jpg", img_four_match);
    imwrite(dir_path + "overlay_" + video_filename + "_Frame_" + to_string(base_frame) + "_" + to_string(base_frame + target_frame) + ".jpg", img_2);
    
    
//    imwrite(dir_path + "Frame_1_out_match.jpg", outimg1);
//    waitKey(0);
    
//    points.clear();
//    for (auto i = transformed_points.begin(); i != transformed_points.end(); i++) {
//        points.push_back(*i);
//    }
//    mask = transformed_mask;
//    overlay = transformed_overlay;
    
    return warp_matrix;
}

int Video_To_Image(string video_filename, bool buffer)
{
    cv::VideoCapture capture(dir_path + video_filename);
    
    if (!capture.isOpened())
        cout << "open video error";
    int frame_width = (int)capture.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    float frame_fps = capture.get(CV_CAP_PROP_FPS);
    int frame_number = capture.get(CV_CAP_PROP_FRAME_COUNT);//总帧数
    cout << "frame_width is " << frame_width<< endl;
    cout << "frame_height is " << frame_height << endl;
    cout << "frame_fps is " << frame_fps << endl;
    cout << "frame_number is " << frame_number << endl;
    
    if (buffer) { return frame_number; }
    
    cv::Mat img;
    for (int i = 0; i < frame_number; i++) {
        cv::Mat frame;
        bool flag = capture.read(frame);
        if (!flag)
        {
            cout << "不能从视频文件读取帧" << endl;
            break;
        }
        imwrite(dir_path + frame_path + video_filename + "_Frame_" + to_string(i) + ".jpg", frame);
    }
    capture.release();
    return frame_number;
}

void Image_To_Video(string video_filename, int frame_num)
{
    cv::VideoWriter writer;
    int isColor = 1;
    int frame_fps = 60;
    int frame_width = 1920;
    int frame_height = 1080;
    using namespace cv;
    string video_name = dir_path + "out.mp4";
    writer = VideoWriter(video_name, CV_FOURCC('X', 'V', 'I', 'D'),frame_fps,Size(frame_width,frame_height),isColor);
    cout << "frame_width is " << frame_width << endl;
    cout << "frame_height is " << frame_height << endl;
    cout << "frame_fps is " << frame_fps << endl;
    
    for (int i = 0; i < frame_num; i++) {
        string s_image_name = dir_path + "overlay_" + video_filename + "_Frame_" + to_string(0) + "_" + to_string(i+1) + ".jpg";
        //        s_image_name = dir_path + "good_match_" + video_filename + "_Frame_" + to_string(0) + "_" + to_string(i+1) + ".jpg";
        Mat img = imread(s_image_name);
        if (!img.data)
            cout << "Could not load image file...\n" << endl;
        writer.write(img);
    }
}


//Mat chartlet(Mat img, Mat overlay){
//
//}


int video(string video_filename){
    int frame_number = Video_To_Image(video_filename, false) - 10;
    
//    int width = 494;
//    int height = 250;
//    int width = 415;
//    int height = 190;
//    int start_x = 600;
//    int start_y = 350;

    
    int width = 245;
    int height = 394;
//    int start_x = 943;
//    int start_y = 300;
    int x1 = 941;
    int y1 = 298;
    int x2 = 1198;
    int y2 = 292;
    int x3 = 945;
    int y3 = 705;
    int x4 = 1196;
    int y4 = 700;

    
    int error_fix = 3;
    
    vector<Point2f> origin_points, transformed_points, mask_points;
    origin_points.push_back(Point2f(0, 0));
    origin_points.push_back(Point2f(width, 0));
    origin_points.push_back(Point2f(0, height));
    origin_points.push_back(Point2f(width, height));
//    transformed_points.push_back(Point2f(start_x, start_y));
//    transformed_points.push_back(Point2f(start_x + width + 7, start_y - 5));
//    transformed_points.push_back(Point2f(start_x, start_y + height));
//    transformed_points.push_back(Point2f(start_x + width, start_y + height));
//    mask_points.push_back(Point2f(start_x + error_fix, start_y + error_fix));
//    mask_points.push_back(Point2f(start_x + width - error_fix, start_y + error_fix));
//    mask_points.push_back(Point2f(start_x + error_fix, start_y + height - error_fix));
//    mask_points.push_back(Point2f(start_x + width - error_fix, start_y + height - error_fix));
    
    
    transformed_points.push_back(Point2f(x1, y1));
    transformed_points.push_back(Point2f(x2, y2));
    transformed_points.push_back(Point2f(x3, y3));
    transformed_points.push_back(Point2f(x4, y4));
    mask_points.push_back(Point2f(x1 + error_fix, y1 + error_fix));
    mask_points.push_back(Point2f(x2 - error_fix, y2 + error_fix));
    mask_points.push_back(Point2f(x3 + error_fix, y3 - error_fix));
    mask_points.push_back(Point2f(x4 - error_fix, y4 - error_fix));
//    transformed_points.push_back(Point2f(397, 215));
//    transformed_points.push_back(Point2f(1065, 200));
//    transformed_points.push_back(Point2f(417, 585));
//    transformed_points.push_back(Point2f(1075, 572));
//    transformed_points.push_back(Point2f(380, 164));
//    transformed_points.push_back(Point2f(1070, 150));
//    transformed_points.push_back(Point2f(395, 529));
//    transformed_points.push_back(Point2f(1078, 525));
    Mat warp_matrix(3,3,CV_32FC1);
    Mat mask_warp_matrix(3,3,CV_32FC1);
    warp_matrix = getPerspectiveTransform(origin_points, transformed_points);
    mask_warp_matrix = getPerspectiveTransform(origin_points, mask_points);
    
//    Mat overlay = imread(dir_path + "Frame.mp4_Frame_38.jpg", IMREAD_UNCHANGED);
    Mat overlay = imread(dir_path + "logo.jpg", IMREAD_UNCHANGED);
    Mat mask = Mat(overlay.rows, overlay.cols, overlay.type(), cv::Scalar::all(255));
//    cout << overlay << endl;
//    cout << mask << endl;
//    imshow("Mask", mask);
//    waitKey(0);
    
    Mat transformed_overlay;
    Mat transformed_mask;
    Mat img = imread(dir_path + frame_path + video_filename + "_Frame_0.jpg", IMREAD_UNCHANGED);
    
    warpPerspective(overlay, transformed_overlay, warp_matrix, img.size());
//    warpPerspective(mask, transformed_mask, warp_matrix, img.size());
    warpPerspective(mask, transformed_mask, mask_warp_matrix, img.size());
    
//    cout << transformed_mask << endl;
    transformed_overlay.copyTo(img, transformed_mask);
//    Mat result = img - img.mul(transformed_mask) + transformed_overlay.mul(transformed_mask);
    
//    imshow("Origin", overlay);
//    imshow("Mask", mask);
//    imshow("Transformed", transformed_overlay);
//    imshow("Transformed mask", transformed_mask);
//    imshow("img", img);
//    imshow("Frame 0 Result", result);
    
    imwrite(dir_path + video_filename + "_Frame_0_overlay.jpg", img);

    for (int i = 0; i < frame_number - 10; i++) {
        cout << "Frame " << to_string(i) << endl;
        frame(video_filename, transformed_points, transformed_overlay, transformed_mask, i);
        cout << endl;
    }

//    for (int i = 0; i < frame_number; i++) {
//        Mat new_overlay = imread(dir_path + "Frame.mp4_Frame_" + to_string(i) + ".jpg", IMREAD_UNCHANGED);
//        Mat new_transformed_overlay;
//        warpPerspective(new_overlay, new_transformed_overlay, warp_matrix, img.size());
//        cout << "Frame " << to_string(i) << endl;
//        frame(video_filename, transformed_points, new_transformed_overlay, transformed_mask, i);
//        cout << endl;
//    }
    Image_To_Video(video_filename, frame_number  - 10);
    return 0;
}


int main(){
//    cout<<CV_VERSION << endl;
    video("long_local.mp4");
//    Video_To_Image("local.mp4", false);
//    Image_To_Video("local.mp4", 280);
}






