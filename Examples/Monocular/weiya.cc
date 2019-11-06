/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include<unistd.h>
#include "M_DataManager.h"

#include "IMU/imudata.h"
#include "IMU/configparam.h"

using namespace std;

int main(int argc, char **argv)
{

    const string imgpath = "/media/navinfo/Bak/Datas/@@1002-0001-190828-00/Output/gray/";
    const string pstpath = "/media/navinfo/Bak/Datas/@@1002-0001-190828-00/Output/gray/pstdatas.txt";
    const string imupath = "/media/navinfo/Bak/Datas/@@1002-0001-190828-00/Output/gray/imudatas.txt";
    const string vocpath = "/media/navinfo/Work/GitHub/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    const string cfgpath = "/media/navinfo/Work/GitHub/LearnVIORB/Examples/Monocular/weiya.yaml";

    vector<double> vTimestamps;
    
    if(!M_DataManager::getSingleton()->LoadData(pstpath,imupath))
    {
        return -1;
    }

    ORB_SLAM2::ConfigParam config(cfgpath);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocpath,cfgpath,ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    size_t nImages = M_DataManager::getSingleton()->GetImgSize();
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;


    // Main loop
    cv::Mat im;
    const int st_no = 600;
    ImgInfoVIter it = M_DataManager::getSingleton()->begin() + st_no;
    ImgInfoVIter ed = M_DataManager::getSingleton()->end();
    M_DataManager::getSingleton()->setIndicator(st_no);
    int index = 0;
    for(; it != ed; ++it)
    {
        //get pic name
        cout << "pic read " << it->first.c_str() << endl;
        // Read image from file
        im = cv::imread(imgpath + it->first,CV_LOAD_IMAGE_UNCHANGED);
        double tframe = it->second._t;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << it->first.c_str() << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        IMURawVector imudatas =  M_DataManager::getSingleton()->getIMUDataFromLastTime(it->second._t);
        std::vector<ORB_SLAM2::IMUData> vimudata;
        for( size_t i = 0; i < imudatas.size(); ++i)
        {
            const ImuRawData &rawdata = imudatas[i];
            ORB_SLAM2::IMUData imudata(rawdata._gyro_x,
                                       rawdata._gyro_y,
                                       rawdata._gyro_z,
                                       rawdata._acc_x,
                                       rawdata._acc_y,
                                       rawdata._acc_z,rawdata._t);
            vimudata.emplace_back(imudata);
        }

        // Pass the image to the SLAM system
        // SLAM.TrackMonocular(im,tframe);
        SLAM.TrackMonoVI(im,vimudata,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[index]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(index < nImages - 1)
            T = (it + 1)->second._t - tframe;
        else if(index > 0)
            T = tframe - (it - 1)->second._t;
        ++index;
        // if(ttrack<T)
        //     usleep(1000);//(T-ttrack)*1e6);
        usleep(2000);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}