#include <condition_variable>
#include <mutex>
#include <stdio.h>

#include <ST/CaptureSession.h>


#include<iostream>
#include <opencv2/opencv.hpp>

#include <string>       // std::string
#include <iostream>     // std::cout
#include <fstream>
#include <sstream>
#include <iomanip>
#include "sys/stat.h"

using namespace std;
using namespace cv;

ofstream img_tsfile;
ofstream acc_tsfile;
ofstream gyo_tsfile;

struct SessionDelegate : ST::CaptureSessionDelegate {
    std::mutex lock;
    std::condition_variable cond;
    bool ready = false;
    bool done = false;

    void captureSessionEventDidOccur(ST::CaptureSession *, ST::CaptureSessionEventId event) override {
        // printf("Received capture session event %d (%s)\n", (int)event, ST::CaptureSessionSample::toString(event));
        switch (event) {
            case ST::CaptureSessionEventId::Ready: {
                std::unique_lock<std::mutex> u(lock);
                ready = true;
                cond.notify_all();
            } break;
            case ST::CaptureSessionEventId::Disconnected:
            case ST::CaptureSessionEventId::EndOfFile:
            case ST::CaptureSessionEventId::Error: {
                std::unique_lock<std::mutex> u(lock);
                done = true;
                cond.notify_all();
            } break;
            default:
                printf("Event %d unhandled\n", (int)event);
        }
    }

    void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) {

        Mat I_g;//(sample.depthFrame.height(), sample.depthFrame.width(), CV_32FC1, sample.depthFrame.depthInMillimeters());
        std::stringstream ss_d;
        std::stringstream ss_g;

        stringstream ss_td;
        stringstream ss_tg;
        string td;
        string tg;

        string s;
        string time_info;
        stringstream ss_t;
        unsigned char* pv;
        float* pd;

        // printf("Received capture session sample of type %d (%s)\n", (int)sample.type, ST::CaptureSessionSample::toString(sample.type));
        switch (sample.type) {
            case ST::CaptureSessionSample::Type::DepthFrame:
                printf("Depth frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
                break;
            case ST::CaptureSessionSample::Type::VisibleFrame:
                printf("Visible frame: size %dx%d\n", sample.visibleFrame.width(), sample.visibleFrame.height());
                break;
            case ST::CaptureSessionSample::Type::InfraredFrame:
                printf("Infrared frame: size %dx%d\n", sample.infraredFrame.width(), sample.infraredFrame.height());
                break;
            case ST::CaptureSessionSample::Type::SynchronizedFrames:
                printf("Synchronized frames: depth %dx%d visible %dx%d infrared %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height(), sample.visibleFrame.width(), sample.visibleFrame.height(), sample.infraredFrame.width(), sample.infraredFrame.height());
                // printf("Depth frame: timestamp %.9f\n",sample.depthFrame.timestamp() );
                // printf("Visible frame: timestamp %.9f\n",sample.visibleFrame.timestamp() );
                // cout << sample.visibleFrame.glProjectionMatrix()<<endl;

                cout << ( sample.visibleFrame.intrinsics() ).cx <<" "<< ( sample.visibleFrame.intrinsics() ).cy <<" "<< ( sample.visibleFrame.intrinsics() ).fx <<" "<< ( sample.visibleFrame.intrinsics() ).fy <<" "<<endl;

                // cout << sample.depthFrame.glProjectionMatrix()<<endl;
                cout << sample.depthFrame.colorCameraPoseInDepthCoordinateFrame()<<endl;
                

                pv = const_cast<unsigned char*>((sample.visibleFrame.undistorted()).yData());
                I_g = cv::Mat(sample.visibleFrame.height(), sample.visibleFrame.width(), CV_8UC1, (void*)pv);

                imshow("window",I_g);
                waitKey(1);

                ss_tg << fixed << setprecision(9)<<sample.visibleFrame.timestamp();
                tg = ss_tg.str();
                ss_d<<"/home/jin/Desktop/data/gray/"<< tg  << ".png";
                s = ss_d.str();
                imwrite( s, I_g );
                ss_d.clear();

                pd = const_cast<float*>( sample.depthFrame.depthInMillimeters() );
                I_g = cv::Mat(sample.depthFrame.height(), sample.depthFrame.width(), CV_32FC1, (void*)pd );
                I_g.convertTo(I_g, CV_16UC1);
                ss_td<< fixed << setprecision(9)<<sample.depthFrame.timestamp();
                td = ss_td.str();
                ss_g<<"/home/jin/Desktop/data/depth/" << tg  << ".png";
                s = "";
                s = ss_g.str();
                imwrite( s, I_g );
                ss_g.clear();

                ss_t << tg << "  "<< "gray/" << tg << ".png  "<< tg << "  "<< "depth/"<< tg << ".png  ";
                time_info = ss_t.str();
                img_tsfile << time_info <<"\n";


                break;
            case ST::CaptureSessionSample::Type::AccelerometerEvent:
                // printf("Accelerometer event: [% .9f %.5f % .5f % .5f]\n", sample.accelerometerEvent.timestamp(), sample.accelerometerEvent.acceleration().x, sample.accelerometerEvent.acceleration().y, sample.accelerometerEvent.acceleration().z);
                
                ss_t << fixed << setprecision(9)<<sample.accelerometerEvent.timestamp()<< " " << setprecision(6)<<sample.accelerometerEvent.acceleration().x<< " " << setprecision(6)<<sample.accelerometerEvent.acceleration().y<< " " << setprecision(6)<<sample.accelerometerEvent.acceleration().z;
                time_info = ss_t.str();
                acc_tsfile << time_info <<"\n";

                break;
            case ST::CaptureSessionSample::Type::GyroscopeEvent:
                // printf("Gyroscope event: [% .9f % .5f % .5f % .5f]\n", sample.gyroscopeEvent.timestamp(), sample.gyroscopeEvent.rotationRate().x, sample.gyroscopeEvent.rotationRate().y, sample.gyroscopeEvent.rotationRate().z);
                
                ss_t << fixed << setprecision(9)<<sample.gyroscopeEvent.timestamp() << " " << setprecision(6)<<sample.gyroscopeEvent.rotationRate().x<< " " << setprecision(6)<<sample.gyroscopeEvent.rotationRate().y<< " " << setprecision(6)<<sample.gyroscopeEvent.rotationRate().z;
                time_info = ss_t.str();
                gyo_tsfile << time_info <<"\n";

                break;
            default:
                printf("Sample type %d unhandled\n", (int)sample.type);
        }
    }

    void waitUntilReady() {
        std::unique_lock<std::mutex> u(lock);
        cond.wait(u, [this]() {
            return ready;
        });
    }

    void waitUntilDone() {
        std::unique_lock<std::mutex> u(lock);
        cond.wait(u, [this]() {
            return done;
        });
    }
};

int main(void) {
	string d_dir = "/home/jin/Desktop/data/"; 
	string d_gry = d_dir + "/gray"; 
	string d_dpt = d_dir + "/depth"; 

	mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	mkdir(d_gry.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  	mkdir(d_dpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	string f_name = d_dir + "/timestamp.txt"; 
	img_tsfile.open(f_name.c_str());

    f_name = "";
    f_name = d_dir + "/acc_timestamp.txt"; 
    acc_tsfile.open(f_name.c_str());

    f_name = "";
    f_name = d_dir + "/gyo_timestamp.txt"; 
    gyo_tsfile.open(f_name.c_str());

    printf("Initialize capture session!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = true;
    settings.structureCore.visibleEnabled = true;
    settings.structureCore.infraredEnabled = true;
    settings.structureCore.accelerometerEnabled = true;
    settings.structureCore.gyroscopeEnabled = true;
    settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
    settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_100Hz;

    SessionDelegate delegate;
    ST::CaptureSession session;
    session.setDelegate(&delegate);

    


    // * @brief Get the exposure and gain of the visible frames from a StructureCore. 
    // void getVisibleCameraExposureAndGain(float* exposure, float* gain) const;

    float*  exposure;
    float*  gain;
    printf("before set ");
    session.setVisibleCameraExposureAndGain(7, 1);
    // session.getVisibleCameraExposureAndGain(exposure,gain);

    cout << exposure <<endl;

    if (!session.startMonitoring(settings)) {
        printf("Failed to initialize capture session\n");
        return 1;
    }

    printf("Waiting for session to become ready...\n");
    delegate.waitUntilReady();
    session.startStreaming();
    delegate.waitUntilDone();
    session.stopStreaming();
    return 0;
}
