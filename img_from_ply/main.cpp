#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/transforms.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::string field, const double low, const double high, const bool remove_inside)
{
    if (low > high)
    {
        std::cout << "Warning! Min is greater than max!\n";
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(low, high);
    pass.setFilterLimitsNegative(remove_inside);
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}

cv::Mat makeImageFromPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string dimensionToRemove, float stepSize1, float stepSize2)
{
    pcl::PointXYZI cloudMin, cloudMax;
    pcl::getMinMax3D(*cloud, cloudMin, cloudMax);

    std::string dimen1, dimen2;
    float dimen1Max, dimen1Min, dimen2Min, dimen2Max;
    if (dimensionToRemove == "x")
    {
        dimen1 = "y";
        dimen2 = "z";
        dimen1Min = cloudMin.y;
        dimen1Max = cloudMax.y;
        dimen2Min = cloudMin.z;
        dimen2Max = cloudMax.z;
    }
    else if (dimensionToRemove == "y")
    {
        dimen1 = "x";
        dimen2 = "z";
        dimen1Min = cloudMin.x;
        dimen1Max = cloudMax.x;
        dimen2Min = cloudMin.z;
        dimen2Max = cloudMax.z;
    }
    else if (dimensionToRemove == "z")
    {
        dimen1 = "x";
        dimen2 = "y";
        dimen1Min = cloudMin.x;
        dimen1Max = cloudMax.x;
        dimen2Min = cloudMin.y;
        dimen2Max = cloudMax.y;
    }

    std::vector<std::vector<int> > pointCountGrid;
    int maxPoints = 0;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> grid;

    for (float i = dimen1Min; i < dimen1Max; i += stepSize1)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr slice = passThroughFilter1D(cloud, dimen1, i, i + stepSize1, false);
        grid.push_back(slice);

        std::vector<int> slicePointCount;

        for (float j = dimen2Min; j < dimen2Max; j += stepSize2)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cell = passThroughFilter1D(slice, dimen2, j, j + stepSize2, false);

            int gridSize = grid_cell->size();
            slicePointCount.push_back(gridSize);

            if (gridSize > maxPoints)
            {
                maxPoints = gridSize;
            }
        }
        pointCountGrid.push_back(slicePointCount);
    }

    cv::Mat mat(static_cast<int>(pointCountGrid.size()), static_cast<int>(pointCountGrid.at(0).size()), CV_8UC1);
    mat = cv::Scalar(0);

    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j < mat.cols; ++j)
        {
            int pointCount = pointCountGrid.at(i).at(j);
            float percentOfMax = (pointCount + 0.0) / (maxPoints + 0.0);
            int intensity = percentOfMax * 255;

            mat.at<uchar>(i, j) = intensity;
        }
    }

//    cv::imshow("test", mat);
//    cv::waitKey(0);
    return mat;
}

void loadPlyAndMakeImage(){
    //boost::shared_ptr<PointCloud<pcl::PointT>> cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::PLYReader Reader;
    //Reader.read("head2.ply", *cloud);
    pcl::io::loadPLYFile("dragon.ply", *cloud);
    std::cout << "loaded" << std::endl;

    cv::Mat image1 = makeImageFromPointCloud(cloud, "x", 0.0003f, 0.0003f);

    //    transformation

//    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//    float theta = M_PI/4; // The angle of rotation in radians
//    transform_1 (0,0) = std::cos (theta);
//    transform_1 (0,1) = -sin(theta);
//    transform_1 (1,0) = sin (theta);
//    transform_1 (1,1) = std::cos (theta);
//
//    transform_1 (0,3) = 0.05;
//
//    printf ("Method #1: using a Matrix4f\n");
//    std::cout << transform_1 << std::endl;
//

//
//    // Executing the transformation
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);
//
//    cv::Mat image2 = makeImageFromPointCloud(cloud, "z", 0.0003f, 0.0003f);

    cv::imshow("trans1", image1);
    //cv::imshow("trans2", image2);
    cv::waitKey(0);
}

int
main (int argc, char** argv)
{
    loadPlyAndMakeImage();
    return (0);
}