#pragma once

#include "ofMain.h"
#include "ofxCaliCam.h"
#include "ofxPDI.h"
#include "ofxGui.h"
class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    ofVideoGrabber vidGrabber;
    
    int camWidth;
    int camHeight;
    cv::Mat raw;
    ofxPDI pdiCam;
    ofxCaliCam caliCam;
    
    ofxPanel gui;
    ofParameter<bool> bEnabled;
    ofParameter<int> fov;
    ofParameter<int> ndisp;
    ofParameter<int> wsize;
    ofParameter<int> thr;
    
    bool bSaving;
    bool bClear;
    ofEasyCam cam;
};
