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
#include "M_Types.h"
#include "Converter.h"
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

    ORB_SLAM2::ConfigParam config(cfgpath);

    VecSet vecs = ReadDmrFile(ORB_SLAM2::ConfigParam::_VelPath);

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

    std::cout << endl << "-------" << endl;
    std::cout << "Start processing sequence ..." << endl;
    std::cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    const int st_no = ORB_SLAM2::ConfigParam::_BeginNo;
    ImgInfoVIter it = M_DataManager::getSingleton()->begin() + st_no;
    ImgInfoVIter ed = M_DataManager::getSingleton()->end();
    M_DataManager::getSingleton()->setIndicator(st_no);
    int index = 0;
    PoseData predata = it->second;
    for(; it != ed; ++it)
    {
        //get pic name
        std::cout << "pic read " << it->first.c_str() << endl;
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

            if( v < 5e-2)
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

        cv::Mat velcity = cv::Mat::eye(4,4,CV_64F);
       
        if(0 != index)
        {//根据post 数据 解算帧间R t
            cv::Mat R;
            cv::Mat t;
            cv::Mat cam2imu = ORB_SLAM2::Converter::toCvDMat(ORB_SLAM2::ConfigParam::GetEigTbc());
            cv::Mat cam2imuR = cam2imu.rowRange(0,3).colRange(0,3);
            cv::Mat cam2imuT = cam2imu.rowRange(0,3).colRange(3,4);
            M_Untils::GetRtFromPose(predata,it->second,cam2imuR,cam2imuT, R,t);

            R.copyTo(velcity.rowRange(0,3).colRange(0,3));
            t.copyTo(velcity.rowRange(0,3).col(3));
            predata = it->second;
        }
        SLAM.TrackMonoVI(im,vimudata,tframe,velcity);

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
        usleep(5.0e5);
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