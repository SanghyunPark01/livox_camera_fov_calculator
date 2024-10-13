#ifndef CALCULATOR_H
#define CALCULATOR_H

#include "livox_camera_fov_calculator/utility.h"

class Calculator
{
private:
    // Param
        // Topic
    std::string LIVOX_TOPIC, IMG_TOPIC;
    bool COMPRESSED_IMG = false;
        // Common
    bool VISUALIZE = false;
    bool SAVE = false;
    std::string SAVE_IMG_PATH, SAVE_RES_PATH;
        // Calibration
    int ROW, COL;
    int TOTAL_PIXEL_NUM;
    cv::Mat CAM_INTRINSIC = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat DISTORTION_MAT = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat EXTRINSIC_T_C_L = cv::Mat::zeros(4, 4, CV_64F);

    // Initialize
    bool _mbIsFirst = false;
    void initExtrinsic(void);

    // ROS
    ros::NodeHandle _nh;
    ros::Subscriber _mSubLivox;
    ros::Subscriber _mSubImg;

    // ROS Callback
    void callbackLivox(const livox_ros_driver::CustomMsg::ConstPtr &msgLivox);
    void callbackImage(const sensor_msgs::ImageConstPtr &msgIMG);
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr &msgIMG);
    std::queue<livox_ros_driver::CustomMsg::ConstPtr> _mqLivoxBuf;
    std::mutex _mtxCallbackLivox;
    std::queue<sensor_msgs::ImageConstPtr> _mqImgBuf;
    std::queue<sensor_msgs::CompressedImageConstPtr> _mqCImgBuf;
    std::mutex _mtxCallbackImg;


    // TimeSync Pair
    double _mdPrevImgTime = 0;
    double _mdCurrImgTime = 0;
    double _mdPrevLivoxTime = 0;
    double _mdCurrLivoxTime = 0;
    std::queue<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZI>>> _mqPairImgLivox;
    std::mutex _mtxSyncedData;
    bool getPair(std::tuple<double, cv::Mat, livox_ros_driver::CustomMsg> &pairImgLivoxMsg);
    void validatePair(std::tuple<double, cv::Mat, livox_ros_driver::CustomMsg> pairImgLivoxMsg);
    
    // Calculate
    cv::Mat _mExtTransVec;
    cv::Mat _mExtRotVec; // Rodrigues
    cv::Mat _mAccumProjectedLivox;
    cv::Mat _micAccumLivoxImg; // for save image
    cv::Mat _migOccupiedImg; //for save image
    bool getData(cv::Mat &icImage, pcl::PointCloud<pcl::PointXYZI> &pclLivoxCloud);
    void projectLivox2Img(cv::Mat &icImage, pcl::PointCloud<pcl::PointXYZI> &pclLivoxCloud);

    // Result
    int _mnOccupiedNum;
    double _mdOccupiedRatio;

    // Debug
    void printParam(void)
    {
        std::cout << "\033[1;32m[TOPIC]\033[0m\n";
        std::cout << "Livox: " << LIVOX_TOPIC << "\nImage: " << IMG_TOPIC << "\nCompressed: " << COMPRESSED_IMG << "\n";
        std::cout << "\033[1;32m[Common]\033[0m\n";
        std::cout << "Visualize: " << VISUALIZE << "\nSave Image Path: " << SAVE_IMG_PATH << "\nSave Result Path: " << SAVE_RES_PATH << "\n";
        std::cout << "\033[1;32m[Calibration]\033[0m\n";
        std::cout << "\033[1;33mCamera Intrinsic\033[0m\n";
        std::cout << CAM_INTRINSIC << "\n";
        std::cout << "\033[1;33mDistortion Coefficients\033[0m\n";
        std::cout << DISTORTION_MAT << "\n";
        std::cout << "\033[1;33mExtrinsic T_c_L\033[0m\n";
        std::cout << EXTRINSIC_T_C_L << "\n";
    }

public:
    Calculator(const ros::NodeHandle& nh_);
    void syncData(void);
    void calculateFOV(void);
    bool saveResult(void)
    {
        if(!SAVE)
        {
            return false;
        }
        // std::string SAVE_IMG_PATH, SAVE_RES_PATH;

        // Save Image
        std::string sBasicImgName = SAVE_IMG_PATH;
        sBasicImgName = SAVE_IMG_PATH.substr(0, SAVE_IMG_PATH.length() - 4);
        
        std::string sAccumLivoxImgName = sBasicImgName + "_accumulate_lidar.png";
        std::string sOccupiedImgName = sBasicImgName + "_occupied.png";

        if(cv::imwrite(sAccumLivoxImgName, _micAccumLivoxImg))std::cout << "Image successfully saved to '" << sAccumLivoxImgName << "'\n";
        else std::cout << "Failed to save the accumulate image.\n";

        if(cv::imwrite(sOccupiedImgName, _migOccupiedImg))std::cout << "Image successfully saved to '" << sOccupiedImgName << "'\n";
        else std::cout << "Failed to save the occupied image.\n";

        // Save Result
        std::ofstream resFile(SAVE_RES_PATH);
        if (resFile.is_open()) {
                resFile << "Pixel Num: "<< TOTAL_PIXEL_NUM << "\n";
                resFile << "Occupied num: " << _mnOccupiedNum << "\n";
                resFile << "Occupied Ratio: " << _mdOccupiedRatio << "\n";
                resFile.close();
                std::cout << "Data successfully written to'" << SAVE_RES_PATH << "'\n";
        }
        else 
        {
            std::cout << "Unable to open file for writing.\n";
        }

        return true;
    }
};


#endif CALCULATOR_H