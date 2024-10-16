#include "area_scan_3d_camera/Camera.h" //q: [0.0, 0.8, -1.6, 0.0, 0.0, 0.0]"
#include "area_scan_3d_camera/api_util.h"
#include "area_scan_3d_camera/parameters/Scanning3D.h"
#include "area_scan_3d_camera/parameters/Scanning2D.h"

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <string>


float background_depth = 1.0;
float background_left = -0.12;
float background_right = 0.0;
float background_up = -0.08;
float background_down = 0.08;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
    // ----------------z-axis---------------------
    if (event.getKeySym () == "a" && event.keyDown ())
    {
    background_depth -= 0.1;
    std::cout << "a was pressed => filter points deeper than " << background_depth << std::endl;
    }

    if (event.getKeySym () == "s" && event.keyDown ())
    {
    background_depth -= 0.01;
    std::cout << "s was pressed => filter points deeper than " << background_depth << std::endl;
    }

    if (event.getKeySym () == "d" && event.keyDown ())
    {
    background_depth += 0.01;
    std::cout << "d was pressed => filter points deeper than " << background_depth << std::endl;
    }

    if (event.getKeySym () == "f" && event.keyDown ())
    {
    background_depth += 0.1;
    std::cout << "f was pressed => filter points deeper than " << background_depth << std::endl;
    }

    // ----------------x-axis-------------------- 
    if (event.getKeySym () == "w" && event.keyDown ())
    {
    background_left -= 0.02;
    std::cout << "w was pressed => filter points lefter than " << background_left << std::endl;
    }

    if (event.getKeySym () == "e" && event.keyDown ())
    {
    background_left += 0.02;
    std::cout << "e was pressed => filter points lefter than " << background_left << std::endl;
    }

    if (event.getKeySym () == "z" && event.keyDown ())
    {
    background_right -= 0.02;
    std::cout << "z was pressed => filter points righter than " << background_right << std::endl;
    }

    if (event.getKeySym () == "x" && event.keyDown ())
    {
    background_right += 0.02;
    std::cout << "x was pressed => filter points righter than " << background_right << std::endl;
    }

    // ----------------y-axis---------------------
    if (event.getKeySym () == "r" && event.keyDown ())
    {
    background_up -= 0.02;
    std::cout << "r was pressed => filter points higher than " << background_up << std::endl;
    }

    if (event.getKeySym () == "t" && event.keyDown ())
    {
    background_up += 0.02;
    std::cout << "t was pressed => filter points higher than " << background_up << std::endl;
    }

    if (event.getKeySym () == "v" && event.keyDown ())
    {
    background_down -= 0.02;
    std::cout << "x was pressed => filter points lower than " << background_down << std::endl;
    }

    if (event.getKeySym () == "b" && event.keyDown ())
    {
    background_down += 0.02;
    std::cout << "c was pressed => filter points lower than " << background_down << std::endl;
    }


    // -----------------save--------------------
    if (event.getKeySym () == "p" && event.keyDown ())
    {
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);

    std::string pcd_name;
    pcd_name = std::to_string(1900 + p->tm_year) + "_" + std::to_string(1 + p->tm_mon) + "_" +
                std::to_string(p->tm_mday) + "_" + std::to_string(p->tm_hour) + "_" + 
                std::to_string(p->tm_min) + "_" + std::to_string(p->tm_sec) + ".pcd";

    std::cout << "p was pressed => save current pcd " << std::endl;
    pcl::io::savePCDFileBinary("/home/zju/realsense_ws/template_pcd/" + pcd_name, *cloud_filtered);
    }
}


void mouseEventOccurred (const pcl::visualization::MouseEvent &event)
{
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
  }
}

pcl::visualization::PCLVisualizer::Ptr interactionCustomizationVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
{

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.1);

    viewer->registerKeyboardCallback (keyboardEventOccurred);

    viewer->registerMouseCallback (mouseEventOccurred);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    
    viewer->setCameraPosition(0.0, 0.0, -1.0, 0, -1.0, 0.0);

    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Alignment Result"));
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    viewer->addCoordinateSystem(0.1);
    //   viewer->initCameraParameters();
    viewer->setCameraPosition(0.0, 0.0, -1.0, 0, -1.0, 0.0);
    return (viewer);
}

namespace {

template <typename T,
          typename = std::enable_if_t<(std::is_same<T, mmind::eye::PointCloud>::value ||
                                       std::is_same<T, mmind::eye::TexturedPointCloud>::value)>>
bool containsInvalidPoint(const T& cloud)
{
    return std::any_of(
        cloud.data(), cloud.data() + cloud.width() * cloud.height() - 1, [](const auto& point) {
            return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
                   std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z);
        });
}

pcl::PCLPointField createPointField(const std::string& name, uint32_t offset, uint8_t datatype,
                                    uint32_t count)
{
    pcl::PCLPointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
}

void convertToPCL(const mmind::eye::PointCloud& cloud,
                  pcl::PointCloud<pcl::PointXYZ>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = cloud.height();
    pclCloud2.width = cloud.width();
    pclCloud2.point_step = sizeof(mmind::eye::PointXYZ);
    pclCloud2.row_step = sizeof(mmind::eye::PointXYZ) * cloud.width();
    pclCloud2.is_dense = !containsInvalidPoint<mmind::eye::PointCloud>(cloud);

    pclCloud2.fields.reserve(3);
    pclCloud2.fields.push_back(createPointField("x", offsetof(mmind::eye::PointXYZ, x),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(mmind::eye::PointXYZ, y),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("z", offsetof(mmind::eye::PointXYZ, z),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * cloud.height());
    memcpy(pclCloud2.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZ*>(cloud.data())),
           (pclCloud2.row_step * cloud.height()));
    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
    return;
}

void convertToPCL(const mmind::eye::TexturedPointCloud& texturedCloud,
                  pcl::PointCloud<pcl::PointXYZRGB>& pclTexturedPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = texturedCloud.height();
    pclCloud2.width = texturedCloud.width();
    pclCloud2.point_step = sizeof(mmind::eye::PointXYZBGR);
    pclCloud2.row_step = sizeof(mmind::eye::PointXYZBGR) * texturedCloud.width();
    pclCloud2.is_dense = !containsInvalidPoint<mmind::eye::TexturedPointCloud>(texturedCloud);

    pclCloud2.fields.reserve(4);
    pclCloud2.fields.push_back(createPointField("x", offsetof(mmind::eye::PointXYZBGR, x),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(mmind::eye::PointXYZBGR, y),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("z", offsetof(mmind::eye::PointXYZBGR, z),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("rgb", offsetof(mmind::eye::PointXYZBGR, rgb),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * texturedCloud.height());
    memcpy(pclCloud2.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZBGR*>(texturedCloud.data())),
           (pclCloud2.row_step * texturedCloud.height()));

    pcl::fromPCLPointCloud2(pclCloud2, pclTexturedPointCloud);
    return;
}
}


int main(int argc, char * argv[])
{
    // Set the camera capture interval to 10 seconds and the total duration of image capturing to 5
    // minutes.
    const auto captureTime = std::chrono::minutes(5);
    const auto capturePeriod = std::chrono::seconds(1);

    mmind::eye::Camera camera;
    if (!findAndConnect(camera))
        return -1;

    mmind::eye::CameraInfo cameraInfo;
    showError(camera.getCameraInfo(cameraInfo));
    printCameraInfo(cameraInfo);

    if (!confirmCapture3D()) {
        camera.disconnect();
        return 0;
    }

    mmind::eye::UserSet& currentUserSet = camera.currentUserSet();

    // Set the exposure times for acquiring depth information.
    showError(currentUserSet.setFloatArrayValue(
        mmind::eye::scanning3d_setting::ExposureSequence::name, std::vector<double>{20, 20}));
    //    showError(currentUserSet.setFloatArrayValue(
    //        mmind::eye::scanning3d_setting::ExposureSequence::name, std::vector<double>{5, 10}));

    // Obtain the current exposure times for acquiring depth information to check if the setting was
    // successful.
    std::vector<double> exposureSequence;
    showError(currentUserSet.getFloatArrayValue(
        mmind::eye::scanning3d_setting::ExposureSequence::name, exposureSequence));
    std::cout << "3D scanning exposure multiplier : " << exposureSequence.size() << "."
              << std::endl;
    for (size_t i = 0; i < exposureSequence.size(); i++) {
        std::cout << "3D scanning exposure time " << i + 1 << ": " << exposureSequence[i] << " ms."
                  << std::endl;
    }

    camera.setPointCloudUnit(mmind::eye::CoordinateUnit::Meter);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = interactionCustomizationVis(cloud_filtered);

    std::cout << "Start capturing for " << captureTime.count() << " minutes." << std::endl;

    const auto start = std::chrono::high_resolution_clock::now();

    // Perform image capturing periodically according to the set interval for the set total
    // duration.
    while (std::chrono::high_resolution_clock::now() - start < captureTime) {
        const auto before = std::chrono::high_resolution_clock::now();

        std::ostringstream ss;
        ss << (std::chrono::duration_cast<std::chrono::seconds>(before - start)).count();
        std::string time = ss.str();

        // cope the camera
        // ------------------------------------------------------------------------------//
        mmind::eye::Frame2DAnd3D frame2DAnd3D;
        showError(camera.capture2DAnd3D(frame2DAnd3D));

        const mmind::eye::PointCloud pointCloud = frame2DAnd3D.frame3D().getUntexturedPointCloud();

        pcl::PointCloud<pcl::PointXYZ> pointCloudPCL(pointCloud.width(), pointCloud.height());
        convertToPCL(pointCloud, pointCloudPCL);

        // filter background
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(pointCloudPCL.makeShared());
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.0, background_depth);
        pass_z.filter(*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(background_left, background_right);
        pass_x.filter(*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(background_up, background_down);
        pass_y.filter(*cloud_filtered);

        // ... and downsampling the point cloud
        const float voxel_grid_size = 0.002f;
        pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
        vox_grid.setInputCloud (cloud_filtered);
        vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
        vox_grid.filter (*cloud_filtered);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_filtered ,255, 0, 0);
        viewer->updatePointCloud<pcl::PointXYZ>(cloud_filtered, target_color, "target cloud");
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

        // ------------------------------------------------------------------------------//

        const auto after = std::chrono::high_resolution_clock::now();
        const auto timeUsed = after - before;
        if (timeUsed < capturePeriod)
            std::this_thread::sleep_for(capturePeriod - timeUsed);
        else {
            std::cout << "The actual capture time is longer than the set capture interval. Please increase "
                         "the capture interval."
                      << std::endl;
        }

        const auto timeRemaining =
            captureTime - (std::chrono::high_resolution_clock::now() - start);
        const auto remainingMinutes =
            std::chrono::duration_cast<std::chrono::minutes>(timeRemaining);
        const auto remainingSeconds =
            std::chrono::duration_cast<std::chrono::seconds>(timeRemaining - remainingMinutes);
        std::cout << "Remaining time: " << remainingMinutes.count() << " minutes and "
                  << remainingSeconds.count() << "seconds." << std::endl;
    }

    std::cout << "Capturing for " << captureTime.count() << " minutes is completed." << std::endl;

    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;
    return 0;
}
