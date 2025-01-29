#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/qos.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>

namespace fs = std::filesystem;

class LidarDatasaver : public rclcpp::Node {
public:
    LidarDatasaver() : Node("lidardatasaver") { 
        RCLCPP_INFO(this->get_logger(), "Lidar datasaver node started.");
        
        // Set directory for files to be saved in
        setLidarTargetDir();

        // Add subscriber for point cloud data
        sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarDatasaver::callbackPointCloud, this, std::placeholders::_1)
        );
    }

    

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
    std::string lidarTargetDir;
    std::ostringstream nextDirName;

    long long nextDirNr = 0;
    long long timestamp;

    // Read point cloud data and save it to seperate files
    void callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Create new directory with meta.yaml
        nextDirName.str("");
        nextDirName << lidarTargetDir << std::setw(8) << std::setfill('0') << nextDirNr << "/" ;
        nextDirNr++;

        fs::create_directories(nextDirName.str());
        timestamp = (std::round(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() * 2) / 2.0);
        createMetaYAML();

        // Get data of sensor
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Save data
        // Create .data file data
        std::ofstream dataFileIntensities(nextDirName.str() + "intensities.data", std::ios::binary);
        std::ofstream dataFilePoints(nextDirName.str() + "points.data", std::ios::binary);
        
        // TODO: add Header to .data files
        
        for (const auto& point : cloud.points) {
            float intensities = point.intensity;
            dataFileIntensities.write(reinterpret_cast<const char*>(&intensities), sizeof(float));

            float coordinates[3] = {point.x, point.y, point.z};
            dataFilePoints.write(reinterpret_cast<const char*>(coordinates), 3 * sizeof(float));
        }
        dataFileIntensities.close();
        dataFilePoints.close();

        int amountPoints = cloud.height * cloud.width;
    
        // Create yaml for data
        createYAML("intensities.yaml", "intensities", "channel", "float", "[" + std::to_string(amountPoints) + ", 1]");
        createYAML("points.yaml", "points", "channel", "float", "[" + std::to_string(amountPoints) + ", 3]");
    
        // TODO: weiter dateien erstellen -> welche Werte werden benötigt?
    }

    // create YAML file
    void createYAML(const std::string& filename, const std::string& name, const std::string& entity, const std::string& datatype, const std::string& shape) {
        YAML::Node yamlNode;
        yamlNode["entity"] = entity;
        yamlNode["data_type"] = datatype;
        yamlNode["type"] = "array";
        yamlNode["shape"] = YAML::Load(shape);
        yamlNode["name"] = name;

        std::ofstream outFile(nextDirName.str() + filename);
        if (outFile.is_open()) {
            outFile << yamlNode;
            outFile.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to create %s", filename.c_str());
        }
    }

    // Create meta.yaml in new directory
    void createMetaYAML() {
        std::ofstream outFile(nextDirName.str() + "meta.yaml");
        if (outFile.is_open()) {
            outFile << "{fov: {phiIncrement: 0.02500000037252903, phiStart: 0, phiStop: 360, thetaIncrement: 0.02471923828125, thetaStart: 30, thetaStop: 130}, programName: 1200 kHz, shock_detected: false, shock_factor: 2.0385180015479287, start_time: " << timestamp << ", entity: sensor_data, type: scan, end_time: " << timestamp << ", num_points: 1665406919, pose_estimation: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], transformation: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}";
            outFile.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to create meta.yaml");
        }
    }

    // Set directory where lidar data should be saved in (created by image_saver package)
    void setLidarTargetDir() {
        std::string baseTargetDir = fs::absolute(fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()).string() + "/sampledata/raw/";

        // Check if a directory was created in the last 5sek
        auto lastCheckTime = std::chrono::system_clock::now() - std::chrono::seconds(5);

        int i = 0;
        while (i < 200) {
            for (const auto& entry : fs::directory_iterator(baseTargetDir)) {
                if (entry.is_directory()) {
                    auto creationTime = std::chrono::time_point_cast<std::chrono::system_clock::duration>(fs::last_write_time(entry) - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
                    
                    // If directory was created (by imagesaver), use that as target dir
                    if (creationTime > lastCheckTime) {
                        lidarTargetDir = entry.path().string() + "/lidar_00000000/";

                        RCLCPP_INFO(this->get_logger(), "Data is saved to %s", lidarTargetDir.c_str());
                        return;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            i++;
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to find target directory.");
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDatasaver>());
    rclcpp::shutdown();
    return 0;
}
