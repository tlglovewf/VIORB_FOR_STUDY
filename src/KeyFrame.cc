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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void KeyFrame::UpdateNavStatePVRFromTcw(const cv::Mat &Tcw,const cv::Mat &Tbc)
{
    unique_lock<mutex> lock(mMutexNavState);
    cv::Mat Twb = Converter::toCvMatInverse(Tbc*Tcw);
    Matrix3d Rwb = Converter::toMatrix3d(Twb.rowRange(0,3).colRange(0,3));
    Vector3d Pwb = Converter::toVector3d(Twb.rowRange(0,3).col(3));

    Matrix3d Rw1 = mNavState.Get_RotMatrix();
    Vector3d Vw1 = mNavState.Get_V();
    Vector3d Vw2 = Rwb*Rw1.transpose()*Vw1;   // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

    mNavState.Set_Pos(Pwb);
    mNavState.Set_Rot(Rwb);
    mNavState.Set_Vel(Vw2);
}

void KeyFrame::SetInitialNavStateAndBias(const NavState& ns)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState = ns;
    // Set bias as bias+delta_bias, and reset the delta_bias term
    mNavState.Set_BiasGyr(ns.Get_BiasGyr()+ns.Get_dBias_Gyr());
    mNavState.Set_BiasAcc(ns.Get_BiasAcc()+ns.Get_dBias_Acc());
    mNavState.Set_DeltaBiasGyr(Vector3d::Zero());
    mNavState.Set_DeltaBiasAcc(Vector3d::Zero());
}

KeyFrame* KeyFrame::GetPrevKeyFrame(void)
{
    unique_lock<mutex> lock(mMutexPrevKF);
    return mpPrevKeyFrame;
}

KeyFrame* KeyFrame::GetNextKeyFrame(void)
{
    unique_lock<mutex> lock(mMutexNextKF);
    return mpNextKeyFrame;
}

void KeyFrame::SetPrevKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexPrevKF);
    mpPrevKeyFrame = pKF;
}

void KeyFrame::SetNextKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexNextKF);
    mpNextKeyFrame = pKF;
}

std::vector<IMUData> KeyFrame::GetVectorIMUData(void)
{
    unique_lock<mutex> lock(mMutexIMUData);
    return mvIMUData;
}

void KeyFrame::AppendIMUDataToFront(KeyFrame* pPrevKF)
{
    std::vector<IMUData> vimunew = pPrevKF->GetVectorIMUData();
    {
        unique_lock<mutex> lock(mMutexIMUData);
        vimunew.insert(vimunew.end(), mvIMUData.begin(), mvIMUData.end());
        mvIMUData = vimunew;
    }
}

void KeyFrame::UpdatePoseFromNS(const cv::Mat &Tbc)
{
    cv::Mat Rbc_ = Tbc.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Pbc_ = Tbc.rowRange(0,3).col(3).clone();

    cv::Mat Rwb_ = Converter::toCvMat(mNavState.Get_RotMatrix());
    cv::Mat Pwb_ = Converter::toCvMat(mNavState.Get_P());

    cv::Mat Rcw_ = (Rwb_*Rbc_).t();
    cv::Mat Pwc_ = Rwb_*Pbc_ + Pwb_;
    cv::Mat Pcw_ = -Rcw_*Pwc_;

    cv::Mat Tcw_ = cv::Mat::eye(4,4,CV_32F);
    Rcw_.copyTo(Tcw_.rowRange(0,3).colRange(0,3));
    Pcw_.copyTo(Tcw_.rowRange(0,3).col(3));

    SetPose(Tcw_);
}

void KeyFrame::UpdateNavState(const IMUPreintegrator& imupreint, const Vector3d& gw)
{
    unique_lock<mutex> lock(mMutexNavState);
    Converter::updateNS(mNavState,imupreint,gw);
}

void KeyFrame::SetNavState(const NavState& ns)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState = ns;
}

const NavState& KeyFrame::GetNavState(void)
{
    unique_lock<mutex> lock(mMutexNavState);
    return mNavState;
}

void KeyFrame::SetNavStateBiasGyr(const Vector3d &bg)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_BiasGyr(bg);
}

void KeyFrame::SetNavStateBiasAcc(const Vector3d &ba)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_BiasAcc(ba);
}

void KeyFrame::SetNavStateVel(const Vector3d &vel)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Vel(vel);
}

void KeyFrame::SetNavStatePos(const Vector3d &pos)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Pos(pos);
}

void KeyFrame::SetNavStateRot(const Matrix3d &rot)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Rot(rot);
}

void KeyFrame::SetNavStateRot(const Sophus::SO3 &rot)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_Rot(rot);
}

void KeyFrame::SetNavStateDeltaBg(const Vector3d &dbg)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_DeltaBiasGyr(dbg);
}

void KeyFrame::SetNavStateDeltaBa(const Vector3d &dba)
{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState.Set_DeltaBiasAcc(dba);
}

const IMUPreintegrator & KeyFrame::GetIMUPreInt(void)
{
    unique_lock<mutex> lock(mMutexIMUData);
    return mIMUPreInt;
}

void KeyFrame::ComputePreInt(void)
{
    unique_lock<mutex> lock(mMutexIMUData);
    if(mpPrevKeyFrame == NULL)
    {
        if(mnId!=0)
        {
            cerr<<"previous KeyFrame is NULL, pre-integrator not changed. id: "<<mnId<<endl;
        }
        return;
    }
    else
    {
        // Debug log
        //cout<<std::fixed<<std::setprecision(3)<<
        //      "gyro bias: "<<mNavState.Get_BiasGyr().transpose()<<
        //      ", acc bias: "<<mNavState.Get_BiasAcc().transpose()<<endl;
        //cout<<std::fixed<<std::setprecision(3)<<
        //      "pre-int terms. prev KF time: "<<mpPrevKeyFrame->mTimeStamp<<endl<<
        //      "pre-int terms. this KF time: "<<mTimeStamp<<endl<<
        //      "imu terms times: "<<endl;

        //check imu data 
        if(mvIMUData.empty())return;
        // Reset pre-integrator first
        mIMUPreInt.reset();

        // IMU pre-integration integrates IMU data from last to current, but the bias is from last
        Vector3d bg = mpPrevKeyFrame->GetNavState().Get_BiasGyr();
        Vector3d ba = mpPrevKeyFrame->GetNavState().Get_BiasAcc();

        // remember to consider the gap between the last KF and the first IMU
        {
            const IMUData& imu = mvIMUData.front();
            double dt = imu._t - mpPrevKeyFrame->mTimeStamp;
            mIMUPreInt.update(imu._g - bg,imu._a - ba,dt);

            // Test log
            if(dt < 0)
            {
                cerr<<std::fixed<<std::setprecision(3)<<"1 dt = "<<dt<<", prev KF vs last imu time: "<<mpPrevKeyFrame->mTimeStamp<<" vs "<<imu._t<<endl;
                std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
            }
            // Debug log
            //cout<<std::fixed<<std::setprecision(3)<<imu._t<<", int dt: "<<dt<<"first imu int since prevKF"<<endl;
        }
        // integrate each imu
        for(size_t i=0; i<mvIMUData.size(); i++)
        {
            const IMUData& imu = mvIMUData[i];
            double nextt;
            if(i==mvIMUData.size()-1)
                nextt = mTimeStamp;         // last IMU, next is this KeyFrame
            else
                nextt = mvIMUData[i+1]._t;  // regular condition, next is imu data

            // delta time
            double dt = nextt - imu._t;
            // update pre-integrator
            mIMUPreInt.update(imu._g - bg,imu._a - ba,dt);

            // Debug log
            //cout<<std::fixed<<std::setprecision(3)<<imu._t<<", int dt: "<<dt<<endl;

            // Test log
            if(dt <= 0)
            {
                cerr<<std::fixed<<std::setprecision(3)<<"dt = "<<dt<<", this vs next time: "<<imu._t<<" vs "<<nextt<<endl;
                std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
            }
        }
    }
    // Debug log
    //cout<<"pre-int delta time: "<<mIMUPreInt.getDeltaTime()<<", deltaR:"<<endl<<mIMUPreInt.getDeltaR()<<endl;
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

KeyFrame::KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB, std::vector<IMUData> vIMUData, KeyFrame* pPrevKF):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mvIMUData = vIMUData;

    //SetNavState(F.GetNavState());

    if(pPrevKF)
    {
        pPrevKF->SetNextKeyFrame(this);
    }
    mpPrevKeyFrame = pPrevKF;
    mpNextKeyFrame = NULL;

    //---------------------------

    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    // Test log
    cerr<<"shouldn't call this KeyFrame()"<<endl;

    mpPrevKeyFrame = NULL;
    mpNextKeyFrame = NULL;

    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();

        for(mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    // Test log
    if(mbBad)
    {
        vector<KeyFrame*> vKFinMap =mpMap->GetAllKeyFrames();
        std::set<KeyFrame*> KFinMap(vKFinMap.begin(),vKFinMap.end());
        if(KFinMap.count(this))
        {
            cerr<<"this bad KF is still in map?"<<endl;
            mpMap->EraseKeyFrame(this);
        }
        mpKeyFrameDB->erase(this);
        // cerr<<"KeyFrame "<<mnId<<" is already bad. Set bad return"<<endl;
        return;
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }

    // Update Prev/Next KeyFrame for prev/next
    KeyFrame* pPrevKF = GetPrevKeyFrame();
    KeyFrame* pNextKF = GetNextKeyFrame();
    if(pPrevKF)
        pPrevKF->SetNextKeyFrame(pNextKF);
    if(pNextKF)
        pNextKF->SetPrevKeyFrame(pPrevKF);
    SetPrevKeyFrame(NULL);
    SetNextKeyFrame(NULL);
    // TODO: this happend once. Log: Current id = 1.
    // Test log.
    if(!pPrevKF) cerr<<"It's culling the first KF? pPrevKF=NULL. Current id: "<<mnId<<endl;
    if(!pNextKF) cerr<<"It's culling the latest KF? pNextKF=NULL. Current id: "<<mnId<<endl;
    // TODO
    if(pPrevKF && pNextKF)
    {
        if(pPrevKF->isBad()) cerr<<"Prev KF isbad in setbad. previd: "<<pPrevKF->mnId<<", current id"<<mnId<<endl;
        if(pNextKF->isBad()) cerr<<"Next KF isbad in setbad. previd: "<<pNextKF->mnId<<", current id"<<mnId<<endl;

        //Debug log, compare the bias of culled KF and the replaced one
        //cout<<"culled KF bg/ba: "<<mNavState.Get_BiasGyr().transpose()<<", "<<mNavState.Get_BiasAcc().transpose()<<endl;
        //cout<<"next KF bg/ba: "<<pNextKF->GetNavState().Get_BiasGyr().transpose()<<", "<<pNextKF->GetNavState().Get_BiasAcc().transpose()<<endl;

        // Update IMUData for NextKF
        pNextKF->AppendIMUDataToFront(this);
        // Re-compute pre-integrator
        pNextKF->ComputePreInt();
    }

    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);

    // Debug log
    //cerr<<"KF set bad, id:"<<mnId<<", connect: "<<pPrevKF->mnId<<" and "<<pNextKF->mnId<<endl;
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
