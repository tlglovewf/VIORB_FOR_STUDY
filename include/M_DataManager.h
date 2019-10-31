//
//  M_DataManager.h
//
//  Created by TuLigen on 2019/10/22.
//  Copyright © 2019年 TuLigen. All rights reserved.
//
#ifndef _M_DATAMANAGER_H_H_
#define _M_DATAMANAGER_H_H_

#include <string>
#include "M_Utils.h"
using namespace std;

typedef std::vector<std::pair<std::string, PoseData>> ImgInfoVector;
typedef ImgInfoVector::iterator ImgInfoVIter;
/*
 * 数据管理类
 */
class M_DataManager
{
public:
    enum PreprocessType
    {
        eStim300
    };

public:
    /*
     * 单例
     */
    static M_DataManager *getSingleton();

    /*
     * 加载数据
     * @param pstpath 预处理后的imgpst 文件
     * @param imupath 预处理后的imu 文件
     */
    bool LoadData(const string &pstpath, const string &imupath);

    /*
    * 获取图片数量
    * 
    */
    size_t GetImgSize()const
    {
        return mPoseDatas.size();
    }
    /* 预处理数据
    * @param imgpath  图片路径
    * @param pstpath  pst路径
    * @param imupath  imu路径
    * @param outpath  输出路径
    * @param oimpath  imu输出路径
    */
    void PreprocessData(const std::string &imgpath,
                        const std::string &pstpath,
                        const std::string &imupath,
                        const std::string &outpath,
                        const std::string &oimpath,
                        PreprocessType type = eStim300);

        /* 获取imgpst 容器头
    */
        ImgInfoVIter begin()
    {
        return mPoseDatas.begin();
    }

    /* 获取imgpst 容器尾
    */
    ImgInfoVIter end()
    {
        return mPoseDatas.end();
    }

    /* 当前时间到上个时间段内的imu数据集合
     * @param cursec(天秒)
     * @return 数据集
     */
    IMURawVector getIMUDataFromLastTime(double cursec);

protected:
    //单例　外部禁用拷贝复制
    M_DataManager() {}
    M_DataManager(const M_DataManager &) {}

protected:
    IMURawVector mIMURawDatas;
    ImgInfoVector mPoseDatas;

    //imu数据量太大  设置游标 提升访问效率
    IMURawVIter mIMURawIndicator;
};

#endif