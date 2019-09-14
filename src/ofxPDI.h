//
//  ofxPDI.hpp
//  devApp
//
//  Created by Dan Moore on 6/3/19.
//

#include "ofMain.h"
#include "ofxOpencv.h"
#include "ofxCv.h"

class ofxPDI{
enum RecMode {
    REC_PERSPECTIVE,
    REC_FISHEYE,
    REC_LONGLAT
};
public:
    void setup(string file, int camWidth, int camHeight);
    void setup(string file);
    void loadParams(string file);
    
    void update(cv::Mat raw_img);
    void draw();
    
    void InitRectifyMap();
    void InitRectifyMap(cv::Mat K, cv::Mat D, double xi0, cv::Mat R,
                        cv::Mat Knew, double xi1, cv::Size size,
                        cv::Mat& map1, cv::Mat& map2, RecMode mode);
    void InitUndistortRectifyMap(cv::Mat K, cv::Mat D, cv::Mat xi, cv::Mat R, cv::Mat P, cv::Size size, cv::Mat& map1, cv::Mat& map2);
    void DisparityImage(const cv::Mat& recl, const cv::Mat& recr, cv::Mat& disp);
    
    void drawScene(const cv::Mat feim, const cv::Mat disp_img);
    int camWidth;
    int camHeight;
    string cam_model;
    int vfov_now;
    int height_now;
    int width_now;
    int wsize_now;
    int ndisp_now;
    bool changed;
    bool is_sgbm;
    cv::Mat Translation;
    cv::Mat Kl;
    cv::Mat Kr;
    cv::Mat Dl;
    cv::Mat Dr;
    cv::Mat xil;
    cv::Mat xir;
    cv::Mat Rl;
    cv::Mat Rr;
    cv::Mat Knew;
    cv::Mat Kfe;
    
    double    xi;
    int cap_cols;
    int cap_rows;
    int img_width;
    int thr_now;
    int rect_cols;
    int rect_rows;
    cv::Mat smap[2][2];
    cv::Mat mmap[2];
    
    cv::Mat fe_img;
    cv::Mat disp_img;
    cv::Mat raw_img;
    cv::Mat raw_imgl;
    cv::Mat raw_imgr;
    cv::Mat rect_imgl;
    cv::Mat rect_imgr;
    
    
    cv::Mat dispGrey;
    ofMesh mesh;
    ofEasyCam cam;
private:
    
};

