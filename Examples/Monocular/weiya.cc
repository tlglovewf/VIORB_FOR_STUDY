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

#include <set>
using namespace std;


typedef std::map<int, double>  VecSet;

/*
 * 读取dmr文件
 */
VecSet ReadDmrFile(const std::string &filepath)
{
    if(!filepath.empty())
    {
        ifstream file;
        file.open(filepath);

        std::string str;

        //read header
        getline(file,str);

        VecSet vecs;
        while(!file.eof())
        {
            getline(file,str);

            double wkt,vel;

            sscanf(str.c_str(), "%lf %lf", &wkt,&vel);

            int t = M_Untils::Wktime2Daytime(wkt);

            // vecs.insert(std::make_pair(t,vel));
            vecs [t] = vel;
        }
        return vecs;
    }
    return VecSet();
}

int main(int argc, char **argv)
{
    const string cfgpath = "/media/navinfo/Work/GitHub/LearnVIORB/Examples/Monocular/weiya.yaml";

    const string velfile = "/media/navinfo/Bak/Datas/@@1002-0001-191107-00/RawData/ROVER/dmr.txt";

    VecSet vecs = ReadDmrFile(velfile);

    ORB_SLAM2::ConfigParam config(cfgpath);

    const string &imgpath = ORB_SLAM2::ConfigParam::_ImgPath;
    const string &pstpath = ORB_SLAM2::ConfigParam::_PstPath;
    const string &imupath = ORB_SLAM2::ConfigParam::_ImuPath;
    const string &vocpath = ORB_SLAM2::ConfigParam::_VocPath;


    vector<double> vTimestamps;
    
    if(!M_DataManager::getSingleton()->LoadData(pstpath,imupath))
    {
        return -1;
    }


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
    const int st_no = ORB_SLAM2::ConfigParam::_BeginNo;
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

            double v = vecs[static_cast<int>(imudatas[i]._t)];

            if( v < 1e-4)
                continue;

            ORB_SLAM2::IMUData imudata(DEG2RAD(rawdata._gyro_x),
                                       DEG2RAD(rawdata._gyro_y),
                                       DEG2RAD(rawdata._gyro_z),
                                       rawdata._acc_x,
                                       rawdata._acc_y,
                                       rawdata._acc_z,rawdata._t);

            vimudata.emplace_back(imudata);
        }

        cout.precision(20);
        cout << " ======== vimudata size ======== " << vimudata.size() << endl;
        // cout << "img data    " << it->second._t << endl;
        // if(!vimudata.empty())
        // {//获取到从上一帧图像时间 到这一帧时间(不包括当前帧) 段内的所有imu数据
        //     cout << "imu data bg " << vimudata.begin()->_t  << endl;
        //     cout << "imu data ed " << vimudata.rbegin()->_t << endl;
        // }
        
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

         if(ttrack<T)
         {
             double ddx = T - ttrack;
            
             usleep(ddx * 1e3);
         }
         //设置长时间等待其他线程执行完成
        //  usleep(1.5e6);
        usleep(1.5e6);
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