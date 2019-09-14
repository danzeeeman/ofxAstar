#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(0, 0, 0);
    camWidth = 2560;  // try to grab at this size.
    camHeight = 960;
    vidGrabber.listDevices();
    vidGrabber.setDeviceID(1);
    vidGrabber.setDesiredFrameRate(30);
    vidGrabber.setup(camWidth, camHeight);
    
    gui.setup("panel");
    gui.add(bEnabled.set("Enabled", true));
    gui.add(fov.set( "fov", 60, 10, 120 ));
    gui.add(ndisp.set( "disp", 7, 0, 20));
    gui.add(wsize.set( "size", 2, 1,  4));
    gui.add(thr.set( "thr", 80, 0, 200));
    
//    cam.setAutoDistance(true);
    cam.setPosition(0, 0, 0);
    cam.setNearClip(0);
    cam.setFarClip(20000);
    bSaving = false;
    bClear = true;
//    pdiCam.setup(ofToDataPath("21-181220-0055.yml"), camWidth, camHeight);
    
    caliCam.setup(ofToDataPath("21-181220-0055.yml"), camWidth, camHeight);
}

//--------------------------------------------------------------
void ofApp::update(){
    if(!bSaving){
        vidGrabber.update();
        
        //    vfov_now = 60;
        //    ndisp_now = 32;
        //    wsize_now = 5;
        
        //    if(ofGetFrameNum()%100==0){
        //        pdiCam.mesh.clear();
        //    }
        
        if(vidGrabber.isFrameNew() && bEnabled.get()){
            
//            if(bClear) pdiCam.mesh.clear();
            
            if(fov.get() != caliCam.vfov_now){
                caliCam.vfov_now = fov.get();
                caliCam.InitRectifyMap();
            }
            if(ndisp.get()*16+16 != pdiCam.ndisp_now){
                pdiCam.ndisp_now = ndisp.get()*16+16;
                caliCam.ndisp_now = ndisp.get()*16+16;
//                pdiCam.InitRectifyMap();
                caliCam.InitRectifyMap();
            }
            if(wsize.get()*2+3 != pdiCam.wsize_now){
                pdiCam.wsize_now = wsize.get()*2+3;
                caliCam.wsize_now = wsize.get()*2+3;
//                pdiCam.InitRectifyMap();
                caliCam.InitRectifyMap();
            }
//            if(thr.get()+200 != pdiCam.thr_now){
//                pdiCam.thr_now = thr.get()*2+3;
//                pdiCam.InitRectifyMap();
       
//            }
            raw = ofxCv::toCv(vidGrabber);
//            pdiCam.update(raw);
            caliCam.update(raw);
        }
    }
    if(bSaving){
        pdiCam.mesh.save(ofToDataPath(ofGetTimestampString()+".ply"), true);
        bSaving = false;
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    caliCam.draw();
//    pdiCam.draw();
    gui.draw();
    
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 's') {
        gui.saveToFile("settings.xml");
    }
    if(key == 'l') {
        gui.loadFromFile("settings.xml");
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    if(key == ' '){
        bSaving = true;
    }
    if(key == 'c'){
        bClear = !bClear;
    }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
