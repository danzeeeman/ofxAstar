//
//  ofxCaliCam.cpp
//  devApp
//
//  Created by Dan Moore on 6/3/19.
//

#include "ofxCaliCam.h"
void ofxCaliCam::setup(string file, int _camWidth, int _camHeight){
    loadParams(file);
    camWidth = _camWidth;
    camHeight = _camHeight;
    
    vfov_now = 60;
    width_now = camWidth/2;
    height_now = camHeight;
    ndisp_now = 32;
    wsize_now = 5;
    

    InitRectifyMap();
}
void ofxCaliCam::setup(string file){
    loadParams(file);
    InitRectifyMap();
}
void ofxCaliCam::loadParams(string file){
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "Failed to open ini parameters" << std::endl;
        return;
    }
    
    cv::Size cap_size;
    fs["cam_model"] >> cam_model;
    fs["cap_size" ] >> cap_size;
    fs["Kl"       ] >> Kl;
    fs["Dl"       ] >> Dl;
    fs["xil"      ] >> xil;
    Rl = cv::Mat::eye(3, 3, CV_64F);
    fs["Rl"       ] >> Rl;
    fs["Kr"       ] >> Kr;
    fs["Dr"       ] >> Dr;
    fs["xir"      ] >> xir;
    fs["Rr"       ] >> Rr;
    fs["T"        ] >> Translation;
    
    fs.release();
    
    img_width = cap_size.width/2;
    cap_cols  = cap_size.width;
    cap_rows  = cap_size.height;
}

void ofxCaliCam::InitUndistortRectifyMap(cv::Mat K, cv::Mat D, cv::Mat xi, cv::Mat R, cv::Mat P, cv::Size size, cv::Mat& map1, cv::Mat& map2){
    map1 = cv::Mat(size, CV_32F);
    map2 = cv::Mat(size, CV_32F);
    
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    double s  = K.at<double>(0,1);
    
    double xid = xi.at<double>(0,0);
    
    double k1 = D.at<double>(0,0);
    double k2 = D.at<double>(0,1);
    double p1 = D.at<double>(0,2);
    double p2 = D.at<double>(0,3);
    
    cv::Mat KRi = (P * R).inv();
    
    for (int i = 0; i < size.height; ++i) {
        double x = i * KRi.at<double>(0,1) + KRi.at<double>(0,2);
        double y = i * KRi.at<double>(1,1) + KRi.at<double>(1,2);
        double w = i * KRi.at<double>(2,1) + KRi.at<double>(2,2);
        for (int j = 0; j < size.width; ++j, x += KRi.at<double>(0,0),
             y += KRi.at<double>(1,0),
             w += KRi.at<double>(2,0)) {
            double r  = sqrt(x * x + y * y + w * w);
            double xs = x / r;
            double ys = y / r;
            double zs = w / r;
            
            double xu = xs / (zs + xid);
            double yu = ys / (zs + xid);
            
            double r2 = xu * xu + yu * yu;
            double r4 = r2 * r2;
            double xd = (1 + k1 * r2 + k2 * r4) * xu + 2 * p1 * xu * yu
            + p2 * (r2 + 2 * xu * xu);
            double yd = (1 + k1 * r2 + k2 * r4) * yu + 2 * p2 * xu * yu
            + p1 * (r2 + 2 * yu * yu);
            
            double u = fx * xd + s * yd + cx;
            double v = fy * yd + cy;
            
            map1.at<float>(i,j) = (float) u;
            map2.at<float>(i,j) = (float) v;
        }
    }
}
void ofxCaliCam::InitRectifyMap() {
    double vfov_rad = vfov_now * CV_PI / 180.;
    double focal = height_now / 2. / tan(vfov_rad / 2.);
    Knew = (cv::Mat_<double>(3, 3) << focal, 0., width_now  / 2. - 0.5,
            0., focal, height_now / 2. - 0.5,
            0., 0., 1.);
    
    cv::Size img_size(width_now, height_now);
    
    InitUndistortRectifyMap(Kl, Dl, xil, Rl, Knew,
                            img_size, smap[0][0], smap[0][1]);
    
    std::cout << "Width: "  << width_now  << "\t"
    << "Height: " << height_now << "\t"
    << "V.Fov: "  << vfov_now   << "\n";
    std::cout << "K Matrix: \n" << Knew << std::endl;
    
    if (cam_model == "stereo") {
        InitUndistortRectifyMap(Kr, Dr, xir, Rr, Knew,
                                img_size, smap[1][0], smap[1][1]);
        std::cout << "Ndisp: " << ndisp_now << "\t"
        << "Wsize: " << wsize_now << "\n";
    }
    std::cout << std::endl;
}

void ofxCaliCam::DisparityImage(const cv::Mat& recl, const cv::Mat& recr, cv::Mat& disp) {
    
    cv::Mat disp16s;
    int N = ndisp_now, W = wsize_now, C = recl.channels();
    if (is_sgbm) {
        cv::Ptr<cv::StereoSGBM> sgbm =
        cv::StereoSGBM::create(0, N, W, 8 * C * W * W, 32 * C * W * W);
        sgbm->compute(recl, recr, disp16s);
    } else {
        cv::Mat grayl, grayr;
        cv::cvtColor(recl, grayl, cv::COLOR_BGR2GRAY);
        cv::cvtColor(recr, grayr, cv::COLOR_BGR2GRAY);
        
        cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(N, W);
        sbm->setPreFilterCap(31);
        sbm->setMinDisparity(0);
        sbm->setTextureThreshold(10);
        sbm->setUniquenessRatio(15);
        sbm->setSpeckleWindowSize(100);
        sbm->setSpeckleRange(32);
        sbm->setDisp12MaxDiff(1);
        sbm->compute(grayl, grayr, disp16s);
    }
    
    double minVal, maxVal;
    minMaxLoc(disp16s, &minVal, &maxVal);
    disp16s.convertTo(disp, CV_8UC1, 255 / (maxVal - minVal));
    
//    How to get the depth map
     double fx = Knew.at<double>(0,0);
     double fy = Knew.at<double>(1,1);
     double cx = Knew.at<double>(0,2);
     double cy = Knew.at<double>(1,2);
     double bl = -Translation.at<double>(0,0);
     
     cv::Mat dispf;
     disp16s.convertTo(dispf, CV_32F, 1.f / 16.f);
     for (int r = 0; r < dispf.rows; ++r) {
         for (int c = 0; c < dispf.cols; ++c) {
         double e = (c - cx) / fx;
         double f = (r - cy) / fy;
         
         double disp  = dispf.at<float>(r,c);
         if (disp <= 0.f)
         continue;
         
         double depth = fx * bl / disp;
         double x = e * depth;
         double y = f * depth;
         double z = depth;
         }
     }
}

void ofxCaliCam::update(cv::Mat _raw_img){
    raw_img = _raw_img;
    raw_imgl = raw_img(cv::Rect(0, 0, img_width, camHeight));
    raw_imgr = raw_img(cv::Rect(img_width, 0, img_width, camHeight));
    
    
    cv::remap(raw_imgl, rect_imgl, smap[0][0], smap[0][1], cv::INTER_LINEAR, 0);
    cv::remap(raw_imgr, rect_imgr, smap[1][0], smap[1][1], cv::INTER_LINEAR, 0);
    
    DisparityImage(rect_imgl, rect_imgr, disp_img);
}
void ofxCaliCam::draw(){
    ofPushMatrix();
    ofScale(0.25, 0.25);
    ofxCv::drawMat(rect_imgl, 0, camHeight);
    ofxCv::drawMat(rect_imgr, camWidth/2, camHeight);
    ofxCv::drawMat(disp_img, 0, 0);
    ofPopMatrix();
}
