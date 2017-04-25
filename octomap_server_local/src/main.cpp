

#include "ColorOctreeImpl.h"
#include "octomap/FeatOctoMapIO.h"
#include "octomap/FeatOcTree.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv/cv.h>

#include <fstream>

using namespace std;
using namespace pcl;

typedef union
{
    struct /*anonymous*/
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;

pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_img, const cv::Mat& rgb_img);
void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);

int main(int argc, char** argv)
{
    //load pcd
    pcl::PointCloud<point_type>::Ptr src (new pcl::PointCloud<point_type>);
    //read input src cloud
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    if(argc == 2)
    {
        if (reader.read (argv[1], *src) < 0)
        {
	        std::cerr << "Failed to read source input pcd file." << std::endl;
	        return (-1);
        }
    }else if(argc >=3)
    {
        cv::Mat img = cv::imread(argv[1], -1);
        cv::Mat dpt = cv::imread(argv[2], -1);
        if(!img.data || !dpt.data)
        {
            cerr<<"Failed to read img file "<<argv[1]<<" or dpt file: "<<argv[2]<<endl; 
            return -1;
        }
        cv::Mat dpt_t; 
        depthToCV8UC1(dpt, dpt_t);        
        src = pointcloud_type::Ptr(createXYZRGBPointCloud(img, dpt_t));
        if(src->points.size() <= 0)
        {
            cerr<<" create pcd has no data!"<<endl;
            return -1;
        }

        cv::namedWindow( "Display img", cv::WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "Display img", img );                   // Show our image inside it.
        cv::namedWindow("Display dpt", cv::WINDOW_AUTOSIZE); 
        cv::imshow("Display dpt", dpt_t);
        cv::waitKey(0);
    }
    
    // output pcd to compare 
    pcl::io::savePCDFile("out.pcd", *src);

    //insert pcd to octree
    ColorOctreeImpl* m_pOctoTree;
    m_pOctoTree = new ColorOctreeImpl(0.05);
    float p[7];
    p[0] = 0;//camTrans.getOrigin().getX();
    p[1] = 0;//camTrans.getOrigin().getY();
    p[2] = 0;//camTrans.getOrigin().getZ();
    p[3] = 0;//camTrans.getRotation().getW();
    p[4] = 0;//camTrans.getRotation().getX();
    p[5] = 0;//camTrans.getRotation().getY();
    p[6] = 0;//camTrans.getRotation().getZ();
    int max_range = 5;
    m_pOctoTree->insertPointCloud(*(src), p, max_range);//octomap should use max_range to define which part will be updated
    m_pOctoTree->updateInnerOccupancy();

    FeatOcTree* feat_tree = dynamic_cast<FeatOcTree*>(m_pOctoTree);

    
    cout<<"OK, finished!"<<endl;
    
    return 0;
}

void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
    //Process images
    if(depth_img.type() == CV_32FC1){
        depth_img.convertTo(mono8_img, CV_8UC1, 100,0); //milimeter (scale of mono8_img does not matter)
    }
    else if(depth_img.type() == CV_16UC1){
        mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
        cv::Mat float_img;
        depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
        depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
        depth_img = float_img;
    }
    else {
        cerr<<"type error!"<<endl;
        // printMatrixInfo(depth_img, "Depth Image");
        // ROS_ERROR_STREAM("Don't know how to handle depth image of type "<< openCVCode2String(depth_img.type()));
    }
}

pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_img, const cv::Mat& rgb_img)
{

    pointcloud_type* cloud (new pointcloud_type() );
    cloud->is_dense         = false; //single point of view, 2d rasterized NaN where no depth value was found

    //xtion 
    float fx = 1./ 577.24;//cam_info->K[0]; //(cloud->width >> 1) - 0.5f;
    float fy = 1./ 580.48;//cam_info->K[4]; //(cloud->width >> 1) - 0.5f;
    float cx = 302.49;//cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
    float cy = 198.71;//cam_info->K[5]; //(cloud->width >> 1) - 0.5f;
    int data_skip_step = 1;
    if(depth_img.rows % data_skip_step != 0 || depth_img.cols % data_skip_step != 0){
        cerr<<"The parameter cloud_creation_skip_step is not a divisor of the depth image dimensions. This will most likely crash the program!"<<endl;;
    }
    cloud->height = ceil(depth_img.rows / static_cast<float>(data_skip_step));
    cloud->width = ceil(depth_img.cols / static_cast<float>(data_skip_step));
    int pixel_data_size = 3;
    bool encoding_bgr = true;
    //Assume BGR
    //char red_idx = 2, green_idx = 1, blue_idx = 0;
    //Assume RGB
    char red_idx = 0, green_idx = 1, blue_idx = 2;
    if(rgb_img.type() == CV_8UC1) pixel_data_size = 1;
    else if(encoding_bgr) { red_idx = 2; blue_idx = 0; }

    unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
    color_pix_step = pixel_data_size * (rgb_img.cols / cloud->width);
    color_row_step = pixel_data_size * (rgb_img.rows / cloud->height -1 ) * rgb_img.cols;
    depth_pix_step = (depth_img.cols / cloud->width);
    depth_row_step = (depth_img.rows / cloud->height -1 ) * depth_img.cols;

    cloud->points.resize (cloud->height * cloud->width);

    // depth_img already has the desired dimensions, but rgb_img may be higher res.
    int color_idx = 0 * color_pix_step - 0 * color_row_step, depth_idx = 0; //FIXME: Hack for hard-coded calibration of color to depth
    double depth_scaling = 0.1; // 1.
    float max_depth = 10; //-1; //MAX_XTION_RANGE;
    float min_depth = 0.5;//-1; //MIN_XTION_RANGE;
    if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();
    
    pointcloud_type::iterator pt_iter = cloud->begin();
    
    ofstream pcd_cor("pcd_coordinate.txt");
    ofstream pix_cor("pix_coordinate.txt");

    for (int v = 0; v < (int)rgb_img.rows; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
    {
        for (int u = 0; u < (int)rgb_img.cols; u += data_skip_step, color_idx += color_pix_step, depth_idx += depth_pix_step, ++pt_iter)
        {
            if(pt_iter == cloud->end()){
                break;
            }
            point_type& pt = *pt_iter;
            if(u < 0 || v < 0 || u >= depth_img.cols || v >= depth_img.rows){
                pt.x = std::numeric_limits<float>::quiet_NaN();
                pt.y = std::numeric_limits<float>::quiet_NaN();
                pt.z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            float Z = depth_img.at<float>(depth_idx) * depth_scaling;
            pix_cor<<u<<"\t"<<v<<"\t"<<Z<<endl;
            
            // Check for invalid measurements
            if (!(Z >= min_depth)) //Should also be trigger on NaN//std::isnan (Z))
            {
                pt.x = (u - cx) * 1.0 * fx; //FIXME: better solution as to act as at 1meter?
                pt.y = (v - cy) * 1.0 * fy;
                pt.z = std::numeric_limits<float>::quiet_NaN();
            }
            else // Fill in XYZ
            {
                pt.x = (u - cx) * Z * fx;
                pt.y = (v - cy) * Z * fy;
                pt.z = Z;
            }
            
            pcd_cor<<pt.x<<"\t"<<pt.y<<"\t"<<pt.z<<endl;

            // Fill in color
            RGBValue color;
            if(color_idx > 0 && color_idx < rgb_img.total()*color_pix_step){ //Only necessary because of the color_idx offset
                if(pixel_data_size == 3){
                    color.Red   = rgb_img.at<uint8_t>(color_idx + red_idx);
                    color.Green = rgb_img.at<uint8_t>(color_idx + green_idx);
                    color.Blue  = rgb_img.at<uint8_t>(color_idx + blue_idx);
                } else {
                    color.Red   = color.Green = color.Blue  = rgb_img.at<uint8_t>(color_idx);
                }
                color.Alpha = 0;
                pt.rgb = color.float_value;
            }
        }
    }

    return cloud;
}


