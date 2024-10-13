#include "livox_camera_fov_calculator/calc.h"

Calculator::Calculator(const ros::NodeHandle& nh_):_nh(nh_)
{
    // Load Param
        // Topic
    _nh.param<std::string>("/topic/livox", LIVOX_TOPIC, "/");
    _nh.param<std::string>("/topic/image", IMG_TOPIC, "/");
    _nh.param<bool>("/topic/img_compressed", COMPRESSED_IMG, false);
        // Common
    _nh.param<bool>("/common/visualize", VISUALIZE, false);
    _nh.param<bool>("/common/save", SAVE, false);
    _nh.param<std::string>("/common/save_img_path", SAVE_IMG_PATH, "/");
    _nh.param<std::string>("/common/save_result_path", SAVE_RES_PATH, "/");
        // Calibration
    std::vector<double> vdTmpCamIntrinsic;
    std::vector<double> vdTmpCamDistortion;
    std::vector<double> vdTmpExtrinsic;
    _nh.param<std::vector<double>>("/calibration/camera_intrinsic", vdTmpCamIntrinsic, std::vector<double>());
    _nh.param<std::vector<double>>("/calibration/dist_coeffs", vdTmpCamDistortion, std::vector<double>());
    _nh.param<std::vector<double>>("/calibration/extrinsic", vdTmpExtrinsic, std::vector<double>());

    // Calibration param to CV matrix
    int nTmpIdx = 0;
    for (int i = 0; i < CAM_INTRINSIC.total(); i++) {
        CAM_INTRINSIC.at<double>(i / CAM_INTRINSIC.cols, i % CAM_INTRINSIC.cols) = vdTmpCamIntrinsic[nTmpIdx++];
    }
    nTmpIdx = 0;
    for (int i = 0; i < DISTORTION_MAT.total(); i++) {
        DISTORTION_MAT.at<double>(i % DISTORTION_MAT.rows, i / DISTORTION_MAT.rows) = vdTmpCamDistortion[nTmpIdx++];
    }
    nTmpIdx = 0;
    for (int i = 0; i < EXTRINSIC_T_C_L.total(); i++) {
        EXTRINSIC_T_C_L.at<double>(i / EXTRINSIC_T_C_L.cols, i % EXTRINSIC_T_C_L.cols) = vdTmpExtrinsic[nTmpIdx++];
    }
    printParam();
    initExtrinsic();


    // ROS
    _mSubLivox = _nh.subscribe(LIVOX_TOPIC, 1000, &Calculator::callbackLivox, this);
    if(COMPRESSED_IMG)
    {
        _mSubImg = _nh.subscribe(IMG_TOPIC, 1000, &Calculator::callbackImageCompressed, this);
    }
    else
    {
        _mSubImg = _nh.subscribe(IMG_TOPIC, 1000, &Calculator::callbackImage, this);
    }
}
void Calculator::initExtrinsic(void)
{
    _mExtTransVec = EXTRINSIC_T_C_L(cv::Rect(3, 0, 1, 3)).clone();
    cv::Mat tmpRotMat = EXTRINSIC_T_C_L(cv::Rect(0,0,3,3)).clone();
    cv::Rodrigues(tmpRotMat, _mExtRotVec);
}


/* @@@@@@@@@@@@@@@@@@@@@
@@@@@@@ Callback @@@@@@@
@@@@@@@@@@@@@@@@@@@@@ */
void Calculator::callbackLivox(const livox_ros_driver::CustomMsg::ConstPtr &msgLivox)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackLivox);
    _mqLivoxBuf.push(msgLivox);
}
void Calculator::callbackImage(const sensor_msgs::ImageConstPtr &msgIMG)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackImg);
    _mqImgBuf.push(msgIMG);
}
void Calculator::callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr &msgIMG)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackImg);
    _mqCImgBuf.push(msgIMG);
}

/* @@@@@@@@@@@@@@@@@@@
@@@@@@@ Thread @@@@@@@
@@@@@@@@@@@@@@@@@@@ */
void Calculator::syncData(void)
{
    while (true)
    {
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

        std::tuple<double, cv::Mat, livox_ros_driver::CustomMsg> pairImgLivoxMsg;
        bool bStatus = getPair(pairImgLivoxMsg);
        if(!bStatus)continue;

        validatePair(pairImgLivoxMsg);
    }
}

void Calculator::calculateFOV(void)
{
    while (true)
    {
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

        cv::Mat icCurrImg;
        pcl::PointCloud<pcl::PointXYZI> pclLivoxCloud;
        bool bStatus = getData(icCurrImg, pclLivoxCloud);
        if(!bStatus)continue;
        
        projectLivox2Img(icCurrImg, pclLivoxCloud);
    }
}

/* @@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@ FOR THREAD @@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@ */
bool Calculator::getPair(std::tuple<double, cv::Mat, livox_ros_driver::CustomMsg> &pairImgLivoxMsg)
{
    if (_mqLivoxBuf.empty() || (COMPRESSED_IMG ? _mqCImgBuf.empty() : _mqImgBuf.empty())) return false;

    {
        std::unique_lock<std::mutex> lockLivox(_mtxCallbackLivox);
        std::unique_lock<std::mutex> lockImage(_mtxCallbackImg);

        // Image
        double dImgTime = 0;
        cv::Mat icCurrImg;
        if(COMPRESSED_IMG)
        {
            auto currImgData = _mqCImgBuf.front();
            dImgTime = currImgData->header.stamp.toSec();
            icCurrImg = convertROSCompressedImage2CvMat(currImgData);
        }
        else
        {
            auto currImgData = _mqImgBuf.front();
            dImgTime = currImgData->header.stamp.toSec();
            icCurrImg = convertROSImage2CvMat(currImgData);
        }

        // Livox
        auto msgLivox = *_mqLivoxBuf.front();
        pairImgLivoxMsg = std::make_tuple(dImgTime, icCurrImg, msgLivox);
    }
    return true;
}
void Calculator::validatePair(std::tuple<double, cv::Mat, livox_ros_driver::CustomMsg> pairImgLivoxMsg)
{
    
    double dImgTime = std::get<0>(pairImgLivoxMsg);
    livox_ros_driver::CustomMsg msgLivoxData = std::get<2>(pairImgLivoxMsg);
    double dLivoxTime = msgLivoxData.header.stamp.toSec();

    // compare time
    _mdCurrImgTime = dImgTime;
    _mdCurrLivoxTime = dLivoxTime;
        // pop image if...
    if(_mdPrevImgTime < dLivoxTime && _mdCurrImgTime < dLivoxTime)
    {
        std::unique_lock<std::mutex> lockImage(_mtxCallbackImg);
        COMPRESSED_IMG ? _mqCImgBuf.pop() : _mqImgBuf.pop();
        _mdPrevImgTime = _mdCurrImgTime;
        return;
    }
        // pop lidar if...
    else if(_mdPrevImgTime > dLivoxTime && _mdCurrImgTime > dLivoxTime)
    {
        std::unique_lock<std::mutex> lockLivox(_mtxCallbackLivox);
        _mqLivoxBuf.pop();
        _mdPrevLivoxTime = _mdCurrLivoxTime;
        return;
    }

    /* --------------------------------------------------------------
        curr status: _mdPrevImgTime < dLiDARTime < _mdCurrImgTime
    -------------------------------------------------------------- */
    _mdPrevImgTime = _mdCurrImgTime;
    _mdPrevLivoxTime = _mdCurrLivoxTime;

    // pop and push
    {
        std::unique_lock<std::mutex> lockLivox(_mtxCallbackLivox);
        std::unique_lock<std::mutex> lockImage(_mtxCallbackImg);
        _mqLivoxBuf.pop();
        COMPRESSED_IMG ? _mqCImgBuf.pop() : _mqImgBuf.pop();
    }
    {
        std::unique_lock<std::mutex> lock(_mtxSyncedData);
        std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZI>> pairData;
        // image
        pairData.first = std::get<1>(pairImgLivoxMsg);
        // livox
        pairData.second = convertLivox2Pcl(msgLivoxData);

        // push
        _mqPairImgLivox.push(pairData);
    }   
}

bool Calculator::getData(cv::Mat &icImage, pcl::PointCloud<pcl::PointXYZI> &pclLivoxCloud)
{
    if(_mqPairImgLivox.empty())return false;
    {
        std::unique_lock<std::mutex> lock(_mtxSyncedData);
        auto pairImgLivoxData = _mqPairImgLivox.front();
        icImage = pairImgLivoxData.first.clone();
        pclLivoxCloud = pairImgLivoxData.second;
        _mqPairImgLivox.pop();

        if(!_mbIsFirst)
        {
            ROW = icImage.rows;
            COL = icImage.cols;
            _mAccumProjectedLivox = cv::Mat::zeros(ROW, COL, icImage.type());
            _mbIsFirst = true;
            std::cout << "\033[1;32m[Image Size]\033[0m\n";
            std::cout << "row: " << ROW << ", col: " << COL << "\n";
            TOTAL_PIXEL_NUM = ROW * COL;
        }
    }
    return true;
}

void Calculator::projectLivox2Img(cv::Mat &icImage, pcl::PointCloud<pcl::PointXYZI> &pclLivoxCloud)
{
    std::vector<cv::Point3f> vpLivoxPoints;
    std::vector<float> vfIntensity;
    std::vector<cv::Point2f> vpProjectedPts;

    for(int i = 0; i < pclLivoxCloud.size(); i++)
    {
        pcl::PointXYZI tmpPt3D = pclLivoxCloud.points[i];
        vpLivoxPoints.emplace_back(cv::Point3f(tmpPt3D.x, tmpPt3D.y, tmpPt3D.z));
        vfIntensity.push_back(tmpPt3D.intensity);
    }

    cv::projectPoints(vpLivoxPoints, _mExtRotVec, _mExtTransVec, CAM_INTRINSIC, DISTORTION_MAT, vpProjectedPts);

    cv::Mat visImgRT = icImage.clone();
    cv::Mat visImgAccum = icImage.clone();
    cv::Mat overlayImg = visImgRT.clone();
    double dAlpha = 0.5;

    int imgWidth = visImgRT.cols;
    int imgHeight = visImgRT.rows;

    for (int i = 0; i < vpProjectedPts.size(); i++) {
        int x = static_cast<int>(std::round(vpProjectedPts[i].x));
        int y = static_cast<int>(std::round(vpProjectedPts[i].y));
        float intensity = vfIntensity[i];
        if (x >= 0 && x < imgWidth && y >= 0 && y < imgHeight) {
            int blue = std::min((float)255.0, (intensity * 255 / 150));
            int red = std::max((float)0.0, 255 - (intensity * 255 / 150));
            int green = 0;     
            overlayImg.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, green, red);
            _mAccumProjectedLivox.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, green, red);
        }
    }

    cv::addWeighted(overlayImg, dAlpha, visImgRT, 1 - dAlpha, 0, visImgRT);
    cv::addWeighted(_mAccumProjectedLivox, dAlpha, visImgAccum, 1 - dAlpha, 0, visImgAccum);
    _micAccumLivoxImg = visImgAccum.clone();

    // Convert Color Img to Binary Image
    cv::Mat grayAccum;
    cv::cvtColor(_mAccumProjectedLivox, grayAccum, cv::COLOR_BGR2GRAY);
    cv::Mat threshImg;
    cv::threshold(grayAccum, threshImg, 1, 255, CV_THRESH_BINARY);
    grayAccum.release();
    
    // Find Contour
    std::vector<std::vector<cv::Point>> vvpContours;
    std::vector<cv::Vec4i> vvecHierarchy;
    cv::findContours(threshImg, vvpContours, vvecHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    threshImg.release();
        // longest contour
    int nMaxLength = 0;
    int nMaxIndex = -1;
    for (int i = 0; i < vvpContours.size(); i++) {
        int nTmpLength = cv::arcLength(vvpContours[i], true); // get length of contour
        if (nTmpLength > nMaxLength) {
            nMaxLength = nTmpLength;
            nMaxIndex = i;
        }
    }

    cv::Mat occupiedImg = cv::Mat::zeros(visImgRT.size(), CV_8UC3);
    if (nMaxIndex >= 0) {
        //
        cv::drawContours(occupiedImg, vvpContours, nMaxIndex, cv::Scalar(255, 255, 255), -1);
        std::vector<cv::Point> vpHull;
        cv::convexHull(vvpContours[nMaxIndex], vpHull);
        polylines(occupiedImg, vpHull, true, cv::Scalar(0, 255, 0), 2);
        //
        std::vector<std::vector<cv::Point>> vvpHulls(1, vpHull);
        cv::Mat tmpigOccupied = cv::Mat::zeros(occupiedImg.size(), CV_8U);;
        cv::fillPoly(tmpigOccupied, vvpHulls, cv::Scalar(255));
        _migOccupiedImg.release();
        _migOccupiedImg = tmpigOccupied.clone();
        tmpigOccupied.release();
    }
    
    _mnOccupiedNum = cv::countNonZero(_migOccupiedImg);
    _mdOccupiedRatio = (double)_mnOccupiedNum/TOTAL_PIXEL_NUM;
    std::cout << "==========================================\n";
    std::cout << "pixel num: " << TOTAL_PIXEL_NUM << "\n";
    std::cout << "Occupied num: " << _mnOccupiedNum << "\n";
    std::cout << "ratio: " << _mdOccupiedRatio << "\n";


    if(VISUALIZE)
    {
        cv::imshow("curr_scan_result", visImgRT);
        cv::imshow("accum_result", visImgAccum);
        cv::imshow("occupied_contour", occupiedImg);
        cv::imshow("occupied_result", _migOccupiedImg);
        cv::waitKey(1);
    }
}