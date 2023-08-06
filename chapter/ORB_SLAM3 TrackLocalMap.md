# TrackLocalMap
通过[TrackWithMotionModel](https://blog.csdn.net/He3he3he/article/details/131649821)或[TrackReferenceKeyFrame](https://blog.csdn.net/He3he3he/article/details/132000817)的`Frame to Frame`跟踪得到了**当前帧的初始位姿**。为了得到更加精准的位姿，可以将**局部地图**投影到当前帧上得到更多的匹配实现`Frame to Map`，然后进一步优化当前帧的位姿
## 1.UpdateLocalMap
`TrackLocalMap`的第一步就是更新当前帧的**局部地图**，该地图包含：
*	通过`UpdateLocalKeyFrames`更新**局部关键帧**：
	1. 能观测到当前帧地图点的**共视关键帧**
	2. `1`中共视关键帧的**共视关键帧**
	3. `1`中共视关键帧的**子关键帧**
	4. `1`中共视关键帧的**父关键帧**
*	通过`UpdateLocalPoints`更新**局部地图点**：
	* 所有局部关键帧的地图点	
```cpp
/**
 * @brief 更新LocalMap
 *
 * 局部地图包括： 
 * 1、K1个关键帧、K2个临近关键帧和参考关键帧
 * 2、由这些关键帧观测到的MapPoints
 */
void Tracking::UpdateLocalMap()
{
    // This is for visualization
    // 设置参考地图点用于绘图显示局部地图点（红色）
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    // 用共视图来更新局部关键帧和局部地图点
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}
```
### UpdateLocalKeyFrames
![在这里插入图片描述](https://img-blog.csdnimg.cn/ae3edfd1b642429bb1471d67f48c582b.png)

1. 遍历当前帧的地图点，统计所有能观测到当前帧地图点的关键帧，这里分为两种情况：
	* IMU未初始化 或者 刚刚完成重定位：使用当前帧的地图点
	* IMU初始化且跟踪不错：使用上一帧的地图点

为什么需要分这两种情况？这主要与`TrackWithMotionModel`、`TrackReferenceKeyFrame`两种跟踪方式有关系。在`TrackWithMotionModel`中如果跟踪不错，则采用`PredictStateIMU`IMU预积分得到当前帧的位姿，而地图点为`NULL`，那么就无法通过当前帧的地图点去寻找共视关键帧。在IMU未初始化或者刚完成重定位时，`TrackWithMotionModel`与`TrackReferenceKeyFrame`通过帧与帧之间的匹配估计当前帧的位姿的同时也会将上一帧或者参考关键帧的地图点赋给当前帧，因此这时使用当前帧的地图点来更新局部地图。
```cpp
    map<KeyFrame*,int> keyframeCounter;
    // 如果IMU未初始化 或者 刚刚完成重定位
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
```
2. 将能观测到当前帧地图点的关键帧(`一级共视关键帧`)添加到局部关键帧`mvpLocalKeyFrames`
```cpp
    // 存储具有最多观测次数（max）的关键帧
    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有3种类型
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();
    // 先申请3倍内存，不够后面再加
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // Step 2.1 类型1：能观测到当前帧地图点的关键帧作为局部关键帧 （将邻居拉拢入伙）（一级共视关键帧）
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        // 如果设定为要删除的，跳过
        if(pKF->isBad())
            continue;

        // 寻找具有最大观测数目的关键帧
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        // 添加到局部关键帧的列表里
        mvpLocalKeyFrames.push_back(pKF);
        // 用该关键帧的成员变量mnTrackReferenceForFrame 记录当前帧的id
        // 表示它已经是当前帧的局部关键帧了，可以防止重复添加局部关键帧
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }
```
3. 将一级共视关键帧的共视关键帧(`二级共视关键帧`)添加到局部关键帧`mvpLocalKeyFrames`
```cpp

        // 类型2:一级共视关键帧的共视（前10个）关键帧，称为二级共视关键帧（将邻居的邻居拉拢入伙）
        // 如果共视帧不足10帧,那么就返回所有具有共视关系的关键帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        // vNeighs 是按照共视程度从大到小排列
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }
```
4. 将一级共视关键帧的`子关键帧`添加到局部关键帧`mvpLocalKeyFrames`
```cpp
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }
```
5. 将一级共视关键帧的`父关键帧`添加到局部关键帧`mvpLocalKeyFrames`
```cpp
        // 类型3:将一级共视关键帧的父关键帧（将邻居的父母们拉拢入伙）
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            // mnTrackReferenceForFrame防止重复添加局部关键帧
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
```
6. **IMU模式下**，添加最近的10帧临时关键帧
```cpp
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&mvpLocalKeyFrames.size()<80)
    {
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }
```
7. 参考帧`mpReferenceKF`标记为与当前帧`共视程度最高`的关键帧
```cpp
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
```
### UpdateLocalPoints
将`mvpLocalKeyFrames`中关键帧的地图点添加到`mvpLocalMapPoints`中
```cpp
void Tracking::UpdateLocalPoints()
{
    // Step 1：清空局部地图点
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    // Step 2：遍历局部关键帧 mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // step 2：将局部关键帧的地图点添加到mvpLocalMapPoints
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            // 用该地图点的成员变量mnTrackReferenceForFrame 记录当前帧的id
            // 表示它已经是当前帧的局部地图点了，可以防止重复添加局部地图点
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}
```
## 2.SearchLocalPoints
通过`UpdateLocalMap`得到了局部地图的地图点很多，需要筛选局部地图中**新增的**在当前帧视野范围内的地图点，投影到当前帧搜索匹配，得到**更多**的匹配关系
1. 标记当前帧中**不是坏点**的地图点
	* `IncreaseVisible`：增加能够观测到该地图点的帧数
	* `mnLastFrameSeen `：该地图点最后被看见的帧
	* `mbTrackInView `：该地图点在视野范围内

在`TrackWithMotionModel`或`TrackReferenceKeyFrame`中通过特征匹配搜索以及优化后，会将地图点中的外点`Outlier`的 `mnLastFrameSeen `标记为`mCurrentFrame.mnId`表示该地图点虽然为外点，但是和当前帧的特征点能匹配上，也就是能够被当前帧看到。在`SearchLocalPoints`中，将当前帧中的地图点的`mnLastFrameSeen `也标记为`mCurrentFrame.mnId`

`mbTrackInView `表示**局部地图中除当前帧已经能够看到的地图点外**的地图点是否在当前帧的视野范围内，所以当前帧的地图点以及`TrackWithMotionModel`或`TrackReferenceKeyFrame`中优化后的外点的`mbTrackInView `标记为`false`
```cpp
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                // 更新能观测到该点的帧数加1(被当前帧观测了)
                pMP->IncreaseVisible();
                // 标记该点被当前帧观测到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // 标记该点在后面搜索匹配时不被投影，因为已经有匹配了
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }
```
2. 对于局部地图中的地图点，判断是否在当前帧的视野范围内
```cpp
    // Step 2：判断所有局部地图点中除当前帧地图点外的点，是否在当前帧视野范围内
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        // 已经被当前帧观测到的地图点肯定在视野范围内，跳过
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        // 跳过坏点
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        // 判断地图点是否在在当前帧视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            // 观测到该点的帧数加1
            pMP->IncreaseVisible();
            // 只有在视野范围内的地图点才参与之后的投影匹配
            nToMatch++;
        }
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }
```
`isInFrustum`主要的作用是判断该地图点是否在当前帧视野范围内
* 检查MapPoint在`当前帧的相机坐标系下`，是否有`正`的深度
* 检查MapPoint投影到当前帧的像素坐标系，是否在图像的有效范围内
* 检查MapPoint到相机中心的距离$norm\left(P_{w}-O_{w}\right)$，是否在`[0.8f*mfMinDistance, 1.2f*mfMaxDistance]`内
* 检查MapPoint的法向量$\frac{\left(P_{w}-O_{w}\right)\cdot P_{n}}{\left|P_{w}-O_{w}\right|\cdot 1}$与`当前相机指向地图点向量`的余弦值，是否小于阈值`viewingCosLimit=0.5`
* 预测尺度：

$$
\begin{array}{c}
\frac{currentDist}{mfMaxDistance}=1.2^{level} \\
level=\left\lceil\log _{1.2}\left(\frac{ currentDist}{mfMaxDistance }\right)\right\rceil
\end{array}
$$
* 检查通过：`mbTrackInView = True`

```cpp
/**
 * @brief 判断路标点是否在视野中
 * 步骤
 * Step 1 获得这个地图点的世界坐标
 * Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，返回false
 * Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
 * Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
 * Step 5 关卡四：计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值, 若小于设定阈值，返回false
 * Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
 * Step 7 记录计算得到的一些参数
 * @param[in] pMP                       当前地图点
 * @param[in] viewingCosLimit           夹角余弦，用于限制地图点和光心连线和法线的夹角
 * @return true                         地图点合格，且在视野内
 * @return false                        地图点不合格，抛弃
 */
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    // 单目，立体匹配双目，rgbd
    if(Nleft == -1)
    {
        // cout << "\na";
		// mbTrackInView是决定一个地图点是否进行重投影的标志
    	// 这个标志的确定要经过多个函数的确定，isInFrustum()只是其中的一个验证关卡。这里默认设置为否
        pMP->mbTrackInView = false;
        pMP->mTrackProjX = -1;
        pMP->mTrackProjY = -1;

        // 3D in absolute coordinates
        // Step 1 获得这个地图点的世界坐标
        Eigen::Matrix<float,3,1> P = pMP->GetWorldPos();

        // 3D in camera coordinates
        // 根据当前帧(粗糙)位姿转化到当前相机坐标系下的三维点Pc
        const Eigen::Matrix<float,3,1> Pc = mRcw * P + mtcw;
        const float Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ = Pc(2);
        const float invz = 1.0f/PcZ;
        // Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，直接返回false
        if(PcZ<0.0f)
            return false;

        const Eigen::Vector2f uv = mpCamera->project(Pc);

        // Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
        // 判断是否在图像边界中，只要不在那么就说明无法在当前帧下进行重投影
        if(uv(0)<mnMinX || uv(0)>mnMaxX)
            return false;
        if(uv(1)<mnMinY || uv(1)>mnMaxY)
            return false;

        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        // Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
     	// 得到认为的可靠距离范围:[0.8f*mfMinDistance, 1.2f*mfMaxDistance]
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        // 得到当前地图点距离当前帧相机光心的距离,注意P，mOw都是在同一坐标系下才可以
    	//  mOw：当前相机光心在世界坐标系下坐标
        const Eigen::Vector3f PO = P - mOw;
        // 取模就得到了距离
        const float dist = PO.norm();

        // 如果不在允许的尺度变化范围内，认为重投影不可靠
        if(dist<minDistance || dist>maxDistance)
            return false;

        // Check viewing angle
        // Step 5 关卡四：
        // 计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值,
        // 若小于cos(viewingCosLimit), 即夹角大于viewingCosLimit弧度则返回
        Eigen::Vector3f Pn = pMP->GetNormal();

        // 计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值，注意平均观测方向为单位向量
        const float viewCos = PO.dot(Pn)/dist;

        // 如果大于给定的阈值 cos(60°)=0.5，认为这个点方向太偏了，重投影不可靠，返回false
        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        // Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // Step 7 记录计算得到的一些参数
        // Data used by the tracking
    	// 通过置位标记 MapPoint::mbTrackInView 来表示这个地图点要被投影 
        pMP->mbTrackInView = true;
        // 该地图点投影在当前图像（一般是左图）的像素横坐标
        pMP->mTrackProjX = uv(0);
        // bf/z其实是视差，相减得到右图（如有）中对应点的横坐标
        pMP->mTrackProjXR = uv(0) - mbf*invz;

        pMP->mTrackDepth = Pc_dist;

        // 该地图点投影在当前图像（一般是左图）的像素纵坐标
        pMP->mTrackProjY = uv(1);
        // 根据地图点到光心距离，预测的该地图点的尺度层级
        pMP->mnTrackScaleLevel= nPredictedLevel;
        // 保存当前视角和法线夹角的余弦值
        pMP->mTrackViewCos = viewCos;

        // 执行到这里说明这个地图点在相机的视野中并且进行重投影是可靠的，返回true
        return true;
    }
    // 左右目时分别验证
    else
    {
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
        pMP -> mnTrackScaleLevel = -1;
        pMP -> mnTrackScaleLevelR = -1;

        pMP->mbTrackInView = isInFrustumChecks(pMP,viewingCosLimit);
        pMP->mbTrackInViewR = isInFrustumChecks(pMP,viewingCosLimit,true);

        return pMP->mbTrackInView || pMP->mbTrackInViewR;
    }
}
```

3  当前帧视野范围内的地图点投影到当前帧，得到更多的匹配
```cpp
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)  // RGBD相机输入的时候,搜索的阈值会变得稍微大一些
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=6;  // 0.4版本这里是3
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        // 如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15
        // 投影匹配得到更多的匹配关系
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
```
### SearchByProjection
`SearchByProjection`通过将局部地图中在当前帧视野范围内的地图点投影到当前帧寻找更多的匹配关系：
1. 对当前帧视野范围内的地图点`mbTrackInView = true`进行投影
```cpp
        // Step 1 遍历有效的局部地图点
        for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
        {
            MapPoint* pMP = vpMapPoints[iMP];
            if(!pMP->mbTrackInView && !pMP->mbTrackInViewR)
                continue;

            if(bFarPoints && pMP->mTrackDepth>thFarPoints)
                continue;

            if(pMP->isBad())
                continue;
```

2. 通过地图点的投影获得当前帧的候选匹配点
	* 若视角夹角较小时, 搜索半径较小
```cpp
                // 通过距离预测的金字塔层数，该层数相对于当前的帧
                const int &nPredictedLevel = pMP->mnTrackScaleLevel;

                // The size of the window will depend on the viewing direction
                // Step 2 设定搜索搜索窗口的大小。取决于视角, 若当前视角和平均视角夹角较小时, r取一个较小的值
                float r = RadiusByViewingCos(pMP->mTrackViewCos);

                // 如果需要扩大范围搜索，则乘以阈值th
                if(bFactor)
                    r*=th;

                // Step 3 通过投影点以及搜索窗口和预测的尺度进行搜索, 找出搜索半径内的候选匹配点索引
                const vector<size_t> vIndices =
                        F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,      // 该地图点投影到一帧上的坐标
                                            r*F.mvScaleFactors[nPredictedLevel],    // 认为搜索窗口的大小和该特征点被追踪到时所处的尺度也有关系
                                            nPredictedLevel-1,nPredictedLevel);     // 搜索的图层范围
```
3. 在候选匹配点中，搜索地图点的最佳与次佳匹配点
```cpp
        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        // 最优的次优的描述子距离和index
        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        // Step 4 寻找候选匹配点中的最佳和次佳匹配点
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            // 如果Frame中的该兴趣点已经有对应的MapPoint了,则退出该次循环
            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            //如果是双目数据
            if(F.Nleft == -1 && F.mvuRight[idx]>0)
            {
                //计算在X轴上的投影误差
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                //超过阈值,说明这个点不行,丢掉.
                //这里的阈值定义是以给定的搜索范围r为参考,然后考虑到越近的点(nPredictedLevel越大), 相机运动时对其产生的影响也就越大,
                //因此需要扩大其搜索空间.
                //当给定缩放倍率为1.2的时候, mvScaleFactors 中的数据是: 1 1.2 1.2^2 1.2^3 ... 
                if(er>r*F.mvScaleFactors[nPredictedLevel])
                    continue;
            }

            const cv::Mat &d = F.mDescriptors.row(idx);

            // 计算地图点和候选投影点的描述子距离
            const int dist = DescriptorDistance(MPdescriptor,d);

            // 寻找描述子距离最小和次小的特征点和索引
            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
                                            : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                            : F.mvKeysRight[idx - F.Nleft].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
                                            : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                            : F.mvKeysRight[idx - F.Nleft].octave;
                bestDist2=dist;
            }
        }
```
4. 条件筛选匹配
	* 最佳匹配距离需要小于阈值`TH_HIGH`
	* 最佳匹配距离与次佳匹配距离小于阈值`mfNNratio`
	* 最佳和次佳不在同一金字塔层级
```cpp
             // 最佳匹配距离还需要满足在设定阈值内
             if(bestDist<=TH_HIGH)
             {
                 // 条件1：bestLevel==bestLevel2 表示 最佳和次佳在同一金字塔层级
                 // 条件2：bestDist>mfNNratio*bestDist2 表示最佳和次佳距离不满足阈值比例。理论来说 bestDist/bestDist2 越小越好
                 if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                     continue;

                 if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                     F.mvpMapPoints[bestIdx]=pMP;

                     if(F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                         F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
                         nmatches++;
                         right++;
                     }

                     nmatches++;
                     left++;
                 }
             }
```
## 3.优化
优化分为两种情况：
* IMU未初始化 或者 刚刚重定位：`PoseOptimization`
* 其他情况：`PoseInertialOptimizationLastFrame`或者`PoseInertialOptimizationLastKeyFrame`
```cpp
    // Step 3：前面新增了更多的匹配关系，BA优化得到更准确的位姿
    int inliers;
    // IMU未初始化，仅优化位姿
    if (!mpAtlas->isImuInitialized())
        Optimizer::PoseOptimization(&mCurrentFrame);
    else
    {
        // 初始化，重定位，重新开启一个地图都会使mnLastRelocFrameId变化
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        else  // 如果积累的IMU数据量比较多，考虑使用IMU数据优化
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            // mbMapUpdated变化见Tracking::PredictStateIMU()
            // 未更新地图
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                // 使用上一普通帧以及当前帧的视觉信息和IMU信息联合优化当前帧位姿、速度和IMU零偏
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                // 使用上一关键帧以及当前帧的视觉信息和IMU信息联合优化当前帧位姿、速度和IMU零偏
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }
```
## PoseOptimization
参考[ORB_SLAM3 TrackWithMotionModel 中的优化](https://blog.csdn.net/He3he3he/article/details/131649821)
## PoseInertialOptimizationLastFrame
`PoseInertialOptimizationLastKeyFrame`主要的作用是利用上一关键帧和当前关键帧的视觉以及IMU信息来优化当前帧的位姿，主要用在`Tracking`的`TrackLocalMap`
![在这里插入图片描述](https://img-blog.csdnimg.cn/5dac29f2628c4571bcd5b239c33626f6.png)
### 输入
|  优化变量         | fixed           |         |
|:--------------|:----------------|:--------|
|  当前帧的$T_{wb}$ | 上一关键帧的$T_{wb}$  | 当前帧的地图点 |
| 当前帧的$b_{a}$   | 上一关键帧的$b_{a}$   |         |
| 当前帧的$b_{g}$   | 上一关键帧的$b_{g}$   |         |
|  当前帧的速度v       | 上一关键帧的速度v        |         |  

### 几种Vertex
#### 1.VertexPose
* 类型：`ImuCamPose`
* 维度：6（3旋转+3平移）
* 优化变量：$T_{wb}$
```cpp
class VertexPose : public g2o::BaseVertex<6,ImuCamPose>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    VertexPose(){}  
    VertexPose(KeyFrame* pKF){  
        setEstimate(ImuCamPose(pKF));  
    }  
    VertexPose(Frame* pF){  
        setEstimate(ImuCamPose(pF));  
    }  
  
  
    virtual bool read(std::istream& is);  
    virtual bool write(std::ostream& os) const;  
  
    // 重置函数,设定被优化变量的原始值  
    virtual void setToOriginImpl() {  
        }  

    // 更新  
    virtual void oplusImpl(const double* update_){  
        _estimate.Update(update_);  
        updateCache();  
    }  
};
```
* 更新$T_{wb}$
`_estimate`是`ImuCamPose`类型，因此会调用`ImuCamPose`的方法`Update`，更新采用**右更新**
$$
\begin{bmatrix}
 R_{wb} & t_{wb}\\
0  & 1
\end{bmatrix} \cdot \begin{bmatrix}
\mathbf{EXP\left (\delta\vec{\phi}\right  )}  & \delta t_{wb}\\
0  & 1
\end{bmatrix} = \begin{bmatrix}
R_{wb}\mathbf{EXP\left (\delta\vec{\phi}\right  )}  & R_{wb}  \delta t_{wb}+ t_{wb}\\
 0 &1
\end{bmatrix}
$$
* 更新$T_{cw}$
$$
T_{cw}=\begin{bmatrix}
R_{cb}  & t_{cb}\\
 0 & 1
\end{bmatrix} \cdot \begin{bmatrix}
 R_{bw} & t_{bw} \\
  0&1
\end{bmatrix} = \begin{bmatrix}
 R_{cb} \cdot R_{bw} & R_{cb} \cdot t_{bw}+ t_{cb}\\
0  & 1
\end{bmatrix}
$$
```cpp
// pu为body坐标系的位姿
void ImuCamPose::Update(const double *pu)  
{  
    Eigen::Vector3d ur, ut;  
    ur << pu[0], pu[1], pu[2];  
    ut << pu[3], pu[4], pu[5];  
  
    // 更新body下的位姿
    twb += Rwb*ut;  
    Rwb = Rwb*ExpSO3(ur);  
  
    // Normalize rotation after 5 updates  
    its++;  
    if(its>=3)  
    {  
        NormalizeRotation(Rwb);  
        its=0;  
    }  
  
    // 更新相机的pose
    const Eigen::Matrix3d Rbw = Rwb.transpose();  
    const Eigen::Vector3d tbw = -Rbw*twb;  
  
    for(int i=0; i<pCamera.size(); i++)  
    {  
        Rcw[i] = Rcb[i]*Rbw;  
        tcw[i] = Rcb[i]*tbw+tcb[i];  
    }  
  
}
```
#### 2.VertexVelocity
* 类型：`Eigen::Vector3d`
* 维度：3
* 优化变量：$v_{x}, v_{y}, v_{z}$
* 更新：

$$
\begin{align}
\tilde{v}_{x} & = v_{x}+\delta v_{x} \\
\tilde{v}_{y} & = v_{y}+\delta v_{y} \\
\tilde{v}_{z} & = v_{z}+\delta v_{z}
\end{align}
$$
```cpp
class VertexVelocity : public g2o::BaseVertex<3,Eigen::Vector3d>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    VertexVelocity(){}  
    VertexVelocity(KeyFrame* pKF);  
    VertexVelocity(Frame* pF);  
  
    virtual bool read(std::istream& is){return false;}  
    virtual bool write(std::ostream& os) const{return false;}  
  
    virtual void setToOriginImpl() {  
        }  
    virtual void oplusImpl(const double* update_){  
	    //获得delta
        Eigen::Vector3d uv;  
        uv << update_[0], update_[1], update_[2];  
        //重新设置估计值
        setEstimate(estimate()+uv);  
    }  
};
```
#### 3.VertexGyroBias
* 类型：`Eigen::Vector3d`
* 维度：3
* 优化变量：$bg_{x}, bg_{y}, bg_{z}$
* 更新：

$$
\begin{align}
\tilde{bg}_{x} & = bg_{x}+\delta bg_{x} \\
\tilde{bg}_{y} & = bg_{y}+\delta bg_{y} \\
\tilde{bg}_{z} & = bg_{z}+\delta bg_{z}
\end{align}
$$
```cpp
class VertexGyroBias : public g2o::BaseVertex<3,Eigen::Vector3d>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    VertexGyroBias(){}  
    VertexGyroBias(KeyFrame* pKF);  
    VertexGyroBias(Frame* pF);  
  
    virtual bool read(std::istream& is){return false;}  
    virtual bool write(std::ostream& os) const{return false;}  
  
    virtual void setToOriginImpl() {  
        }  
    virtual void oplusImpl(const double* update_){  
	    //获取delta
        Eigen::Vector3d ubg;  
        ubg << update_[0], update_[1], update_[2];  
        //更新，重新设置估计
        setEstimate(estimate()+ubg);  
    }  
};
```
#### 4.VertexAccBias
* 类型：`Eigen::Vector3d`
* 维度：3
* 优化变量：$ba_{x}, ba_{y}, ba_{z}$
* 更新：

$$
\begin{align}
\tilde{ba}_{x} & = ba_{x}+\delta ba_{x} \\
\tilde{ba}_{y} & = ba_{y}+\delta ba_{y} \\
\tilde{ba}_{z} & = ba_{z}+\delta ba_{z}
\end{align}
$$
```cpp
class VertexAccBias : public g2o::BaseVertex<3,Eigen::Vector3d>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    VertexAccBias(){}  
    VertexAccBias(KeyFrame* pKF);  
    VertexAccBias(Frame* pF);  
  
    virtual bool read(std::istream& is){return false;}  
    virtual bool write(std::ostream& os) const{return false;}  
  
    virtual void setToOriginImpl() {  
        }  
    virtual void oplusImpl(const double* update_){  
        Eigen::Vector3d uba;  
        uba << update_[0], update_[1], update_[2];  
        setEstimate(estimate()+uba);  
    }  
};
```
### 几种边
#### 1.EdgeMonoOnlyPose
* 属性：**一元边**
* 观测：$p$
* 优化变量：$T_{wb}$
* 残差：$error = p-\pi \left(T_{cw} \cdot P_{w}\right)$
```cpp
//2 观测的维度
//Eigen::Vector2d观测的类型
//VertexPose 顶点的类型
class EdgeMonoOnlyPose : public g2o::BaseUnaryEdge<2,Eigen::Vector2d,VertexPose>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  

   //传入预先的值
    EdgeMonoOnlyPose(const cv::Mat &Xw_, int cam_idx_=0):Xw(Converter::toVector3d(Xw_)),  
        cam_idx(cam_idx_){}  
  
    virtual bool read(std::istream& is){return false;}  
    virtual bool write(std::ostream& os) const{return false;}  

	// 重投影残差 
    void computeError(){  
	    // 获取顶点
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);  
        // 获取观测
        const Eigen::Vector2d obs(_measurement);  
        // 计算残差
        _error = obs - VPose->estimate().Project(Xw,cam_idx);  
    }  
  
    virtual void linearizeOplus();  
  
    bool isDepthPositive()  
    {  
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);  
        return VPose->estimate().isDepthPositive(Xw,cam_idx);  
    }  

	//获得hessian矩阵
    Eigen::Matrix<double,6,6> GetHessian(){  
        linearizeOplus();  
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;  
    }  
  
public:  
    const Eigen::Vector3d Xw;  
    const int cam_idx;  
};
```

* jacobian
$$
\begin{array}{l}
T_{cw}T_{wb}exp\left (\delta \xi^{\wedge }\right)P_{b} -T_{cw}T_{wb}P_{b} \\
= T_{cw}T_{wb}\left (I+\xi^{\wedge }\right)P_{b} -T_{cw}T_{wb}P_{b} \\
= T_{cw}T_{wb}\xi^{\wedge }P_{b} \\
=\begin{bmatrix}
R_{cw}  & t_{cw} \\
 0 & 1
\end{bmatrix} \cdot \begin{bmatrix}
R_{wb}  & t_{wb} \\
  0&1
\end{bmatrix} \cdot \begin{bmatrix}
 \delta \phi^{\wedge } & \delta t \\ 
  0& 1
\end{bmatrix}P_{b} \\
= \begin{bmatrix}
 R_{cw}\cdot R_{wb} & R_{cw}\cdot t_{wb}+t_{cw}\\
 0 & 1
\end{bmatrix} \cdot \begin{bmatrix}
 \delta \phi^{\wedge } & \delta t \\ 
  0& 1
\end{bmatrix}P_{b} \\
= \begin{bmatrix}
  R_{cw}\cdot R_{wb} \delta \phi^{\wedge }& R_{cw}\cdot R_{wb}\delta t+R_{cw}\cdot t_{wb}+t_{cw}\\
 0 & 1
\end{bmatrix}P_{b} \\
=\begin{bmatrix}
R_{cw}\cdot R_{wb} \delta \phi^{\wedge }P_{b}+R_{cw}\cdot R_{wb}\delta t+R_{cw}\cdot t_{wb}+t_{cw} \\
1
\end{bmatrix} \\
= \begin{bmatrix}
\underbrace{-R_{cw}\cdot R_{wb}  P_{b}^{\wedge }\delta \phi}_{\delta \phi} +\underbrace{R_{cw}\cdot R_{wb}\delta t+R_{cw}\cdot t_{wb}+t_{cw}}_{\delta  t}  \\
1
\end{bmatrix} \\
\Rightarrow \begin{bmatrix}
-R_{cb} P_{b}^{\wedge }  & R_{cb} \\
0  & 0
\end{bmatrix}
\end{array}
$$
```cpp
void EdgeMonoOnlyPose::linearizeOplus()  
{  
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);  
  
    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];  
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];  
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;  
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];  
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];  
  
    Eigen::Matrix<double,2,3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);  
  
    Eigen::Matrix<double,3,6> SE3deriv;  
    double x = Xb(0);  
    double y = Xb(1);  
    double z = Xb(2);  
    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,  
            -z , 0.0, x, 0.0, 1.0, 0.0,  
            y ,  -x , 0.0, 0.0, 0.0, 1.0;  
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv; // symbol different becasue of update mode  
}
```
#### 2.EdgeInertial
```cpp
//9 观测的维度， Vector9d观测的类型
class EdgeInertial : public g2o::BaseMultiEdge<9,Vector9d>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  
    EdgeInertial(IMU::Preintegrated* pInt);  
  
    virtual bool read(std::istream& is){return false;}  
    virtual bool write(std::ostream& os) const{return false;}  
  
    void computeError();  
    virtual void linearizeOplus();  
  
    // 关于pose1与2 的旋转平移速度，以及之间的偏置的信息矩阵  
    Eigen::Matrix<double,24,24> GetHessian(){  
        linearizeOplus();  
        Eigen::Matrix<double,9,24> J;  
        J.block<9,6>(0,0) = _jacobianOplus[0];  
        J.block<9,3>(0,6) = _jacobianOplus[1];  
        J.block<9,3>(0,9) = _jacobianOplus[2];  
        J.block<9,3>(0,12) = _jacobianOplus[3];  
        J.block<9,6>(0,15) = _jacobianOplus[4];  
        J.block<9,3>(0,21) = _jacobianOplus[5];  
        return J.transpose()*information()*J;  
    }  
  
    // 没用  
    Eigen::Matrix<double,18,18> GetHessianNoPose1(){  
        linearizeOplus();  
        Eigen::Matrix<double,9,18> J;  
        J.block<9,3>(0,0) = _jacobianOplus[1];  
        J.block<9,3>(0,3) = _jacobianOplus[2];  
        J.block<9,3>(0,6) = _jacobianOplus[3];  
        J.block<9,6>(0,9) = _jacobianOplus[4];  
        J.block<9,3>(0,15) = _jacobianOplus[5];  
        return J.transpose()*information()*J;  
    }  
  
    // 关于pose2 的旋转平移信息矩阵  
    Eigen::Matrix<double,9,9> GetHessian2(){  
        linearizeOplus();  
        Eigen::Matrix<double,9,9> J;  
        J.block<9,6>(0,0) = _jacobianOplus[4];  
        J.block<9,3>(0,6) = _jacobianOplus[5];  
        return J.transpose()*information()*J;  
    }  
  
    // 预积分中对应的状态对偏置的雅可比  
    const Eigen::Matrix3d JRg, JVg, JPg;  
    const Eigen::Matrix3d JVa, JPa;  
  
    IMU::Preintegrated* mpInt;  // 预积分  
    const double dt;  // 预积分时间  
    Eigen::Vector3d g;  // 0, 0, -IMU::GRAVITY_VALUE  
};
```

* 残差

$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{R}_{i j}} & \triangleq \log \left\{\left[\Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)\right]^{T} \cdot \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right\} \\
& \triangleq \log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \Delta \mathbf{R}_{i j}\right] \\
\mathbf{r}_{\Delta \mathbf{v}_{i j}} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\left[\Delta \tilde{\mathbf{v}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}\right] \\
& \triangleq \Delta \mathbf{v}_{i j}-\Delta \hat{\mathbf{v}}_{i j} \\
\mathbf{r}_{\Delta \mathbf{p}_{i j}} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\left[\Delta \tilde{\mathbf{p}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}\right] \\
& \triangleq \Delta \mathbf{p}_{i j}-\Delta \hat{\mathbf{p}}_{i j}
\end{aligned}
$$

```cpp
void EdgeInertial::computeError()  
{  
    // 获得顶点的信息
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);  
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);  
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);  
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);  
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);  
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);  

	//获得imu的bias
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);  

	//计算 dR, dV, dP
    const Eigen::Matrix3d dR = Converter::toMatrix3d(mpInt->GetDeltaRotation(b1));  
    const Eigen::Vector3d dV = Converter::toVector3d(mpInt->GetDeltaVelocity(b1));  
    const Eigen::Vector3d dP = Converter::toVector3d(mpInt->GetDeltaPosition(b1));  

	//计算残差
    const Eigen::Vector3d er = LogSO3(dR.transpose()*VP1->estimate().Rwb.transpose()*VP2->estimate().Rwb);  
    const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose()*(VV2->estimate() - VV1->estimate() - g*dt) - dV;  
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose()*(VP2->estimate().twb - VP1->estimate().twb  
                                                               - VV1->estimate()*dt - g*dt*dt/2) - dP;  
  
    _error << er, ev, ep;  
}
```
* jacobian：[ORB_SLAM3_IMU预积分理论推导(残差)](https://blog.csdn.net/He3he3he/article/details/130054238?spm=1001.2014.3001.5502)
```cpp
void EdgeInertial::linearizeOplus()  
{  
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);  
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);  
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);  
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);  
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);  
    const VertexVelocity* VV2= static_cast<const VertexVelocity*>(_vertices[5]);  
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);  
    const IMU::Bias db = mpInt->GetDeltaBias(b1);  
    Eigen::Vector3d dbg;  
    dbg << db.bwx, db.bwy, db.bwz;  
  
    const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb; // Ri  
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();    // Ri.t()  
    const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb; // Rj  
  
    const Eigen::Matrix3d dR = Converter::toMatrix3d(mpInt->GetDeltaRotation(b1));  
    const Eigen::Matrix3d eR = dR.transpose() * Rbw1 * Rwb2;   // r△Rij  
    const Eigen::Vector3d er = LogSO3(eR);                     // r△φij  
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er); // Jr^-1(log(△Rij))  
  
    // 就很神奇，_jacobianOplus个数等于边的个数，里面的大小等于观测值维度（也就是残差）× 每个节点待优化值的维度  
    // Jacobians wrt Pose 1  
    // _jacobianOplus[0] 9*6矩阵 总体来说就是三个残差分别对pose1的旋转与平移（p）求导  
    _jacobianOplus[0].setZero();  
     // rotation  
    // (0,0)起点的3*3块表示旋转残差对pose1的旋转求导  
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1; // OK  
    // (3,0)起点的3*3块表示速度残差对pose1的旋转求导  
    _jacobianOplus[0].block<3,3>(3,0) = Skew(Rbw1*(VV2->estimate() - VV1->estimate() - g*dt)); // OK  
    // (6,0)起点的3*3块表示位置残差对pose1的旋转求导  
    _jacobianOplus[0].block<3,3>(6,0) = Skew(Rbw1*(VP2->estimate().twb - VP1->estimate().twb  
                                                   - VV1->estimate()*dt - 0.5*g*dt*dt)); // OK  
    // translation    // (6,3)起点的3*3块表示位置残差对pose1的位置求导  
    _jacobianOplus[0].block<3,3>(6,3) = -Eigen::Matrix3d::Identity(); // OK  
  
    // Jacobians wrt Velocity 1    // _jacobianOplus[1] 9*3矩阵 总体来说就是三个残差分别对pose1的速度求导  
    _jacobianOplus[1].setZero();  
    _jacobianOplus[1].block<3,3>(3,0) = -Rbw1; // OK  
    _jacobianOplus[1].block<3,3>(6,0) = -Rbw1*dt; // OK  
  
    // Jacobians wrt Gyro 1    // _jacobianOplus[2] 9*3矩阵 总体来说就是三个残差分别对陀螺仪偏置的速度求导  
    _jacobianOplus[2].setZero();  
    _jacobianOplus[2].block<3,3>(0,0) = -invJr*eR.transpose()*RightJacobianSO3(JRg*dbg)*JRg; // OK  
    _jacobianOplus[2].block<3,3>(3,0) = -JVg; // OK  
    _jacobianOplus[2].block<3,3>(6,0) = -JPg; // OK  
  
    // Jacobians wrt Accelerometer 1    // _jacobianOplus[3] 9*3矩阵 总体来说就是三个残差分别对加速度计偏置的速度求导  
    _jacobianOplus[3].setZero();  
    _jacobianOplus[3].block<3,3>(3,0) = -JVa; // OK  
    _jacobianOplus[3].block<3,3>(6,0) = -JPa; // OK  
  
    // Jacobians wrt Pose 2    // _jacobianOplus[4] 9*6矩阵 总体来说就是三个残差分别对pose2的旋转与平移（p）求导  
    _jacobianOplus[4].setZero();  
    // rotation  
    _jacobianOplus[4].block<3,3>(0,0) = invJr; // OK  
    // translation    _jacobianOplus[4].block<3,3>(6,3) = Rbw1*Rwb2; // OK  
  
    // Jacobians wrt Velocity 2    // _jacobianOplus[5] 9*3矩阵 总体来说就是三个残差分别对pose2的速度求导  
    _jacobianOplus[5].setZero();  
    _jacobianOplus[5].block<3,3>(3,0) = Rbw1; // OK  
}
```
#### 3.EdgeGyroRW
```cpp
class EdgeGyroRW : public g2o::BaseBinaryEdge<3,Eigen::Vector3d,VertexGyroBias,VertexGyroBias>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  
    EdgeGyroRW(){}  
  
    virtual bool read(std::istream& is){return false;}  
    virtual bool write(std::ostream& os) const{return false;}  
  
    void computeError(){  
        const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[0]);  
        const VertexGyroBias* VG2= static_cast<const VertexGyroBias*>(_vertices[1]);  
        _error = VG2->estimate()-VG1->estimate();  
    }  
  
    virtual void linearizeOplus(){  
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();  
        _jacobianOplusXj.setIdentity();  
    }  
  
    Eigen::Matrix<double,6,6> GetHessian(){  
        linearizeOplus();  
        Eigen::Matrix<double,3,6> J;  
        J.block<3,3>(0,0) = _jacobianOplusXi;  
        J.block<3,3>(0,3) = _jacobianOplusXj;  
        return J.transpose()*information()*J;  
    }  
  
    Eigen::Matrix3d GetHessian2(){  
        linearizeOplus();  
        return _jacobianOplusXj.transpose()*information()*_jacobianOplusXj;  
    }  
};
```
#### 4.EdgeAccRW
```cpp
class EdgeAccRW : public g2o::BaseBinaryEdge<3,Eigen::Vector3d,VertexAccBias,VertexAccBias>  
{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  
    EdgeAccRW(){}  
  
    virtual bool read(std::istream& is){return false;}  
    virtual bool write(std::ostream& os) const{return false;}  
  
    void computeError(){  
        const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[0]);  
        const VertexAccBias* VA2= static_cast<const VertexAccBias*>(_vertices[1]);  
        _error = VA2->estimate()-VA1->estimate();  
    }  
  
    virtual void linearizeOplus(){  
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();  
        _jacobianOplusXj.setIdentity();  
    }  
  
    Eigen::Matrix<double,6,6> GetHessian(){  
        linearizeOplus();  
        Eigen::Matrix<double,3,6> J;  
        J.block<3,3>(0,0) = _jacobianOplusXi;  
        J.block<3,3>(0,3) = _jacobianOplusXj;  
        return J.transpose()*information()*J;  
    }  
  
    Eigen::Matrix3d GetHessian2(){  
        linearizeOplus();  
        return _jacobianOplusXj.transpose()*information()*_jacobianOplusXj;  
    }  
};
```
### 步骤一：初始化
```cpp
g2o::SparseOptimizer optimizer;  
g2o::BlockSolverX::LinearSolverType * linearSolver;  
  
// 使用dense的求解器，（常见非dense求解器有cholmod线性求解器和shur补线性求解器）  
linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();  
  
g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);  
//使用高斯牛顿求解器  
g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);  
optimizer.setVerbose(false);  
optimizer.setAlgorithm(solver);
```
此处的块求解器为`BlockSolverX`
```cpp
typedef BlockSolver< BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> > BlockSolverX;
```

### 步骤二：设置Vertex
#### 当前帧
*  Pose：[VertexPose](ORB_SLAM3_G2oType.md#VertexPose)
```cpp
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);
```
* 速度：[VertexVelocity](ORB_SLAM3_G2oType.md#VertexVelocity)
```cpp
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
```
* 陀螺仪bias：[VertexGyroBias](ORB_SLAM3_G2oType.md#VertexGyroBias)
```cpp
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
```
* 加速度bias：[VertexAccBias](ORB_SLAM3_G2oType.md#VertexAccBias)
```cpp
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);
```
#### 上一关键帧
```cpp
    KeyFrame* pKF = pFrame->mpLastKeyFrame;
    VertexPose* VPk = new VertexPose(pKF);
    VPk->setId(4);
    VPk->setFixed(true);
    optimizer.addVertex(VPk);
    
    VertexVelocity* VVk = new VertexVelocity(pKF);
    VVk->setId(5);
    VVk->setFixed(true);
    optimizer.addVertex(VVk);
    
    VertexGyroBias* VGk = new VertexGyroBias(pKF);
    VGk->setId(6);
    VGk->setFixed(true);
    optimizer.addVertex(VGk);
    
    VertexAccBias* VAk = new VertexAccBias(pKF);
    VAk->setId(7);
    VAk->setFixed(true);
    optimizer.addVertex(VAk);
```
### 步骤三：设置edge
* 对于当前帧的每个地图点，添加边[EdgeMonoOnlyPose](ORB_SLAM3_G2oType.md#EdgeMonoOnlyPose)
	* 设置vertex：`VertexPose`
	* 设置观测值
	* 设置信息矩阵
	* 设置鲁棒核函数
```cpp
//观测
Eigen::Matrix<double,2,1> obs;  
obs << kpUn.pt.x, kpUn.pt.y;  
  
//创建边(地图点， 相机id)
EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);  
  
//设置vertex
e->setVertex(0,VP);  
  
//设置观测值
e->setMeasurement(obs);  
  
// 获取不确定度 
const float unc2 = pFrame->mpCamera->uncertainty2(obs);  
  
//invSigma2 = (Inverse(协方差矩阵))^2，表明该约束在各个维度上的可信度  
// 图像金字塔层数越高，可信度越差  
const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;  

//设置该约束的信息矩阵  
e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);  

// 设置鲁棒核函数
g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;  
e->setRobustKernel(rk);  
  
//重投影误差的自由度为2，设置对应的卡方阈值  
rk->setDelta(thHuberMono);  
  
//将第一种边加入优化器  
optimizer.addEdge(e);
```
* [EdgeInertial](ORB_SLAM3_G2oType.md#EdgeInertial)
	* 设置vertex:
		* 当前帧：VP, VV
		* 上一帧：VPk, VVk, VGk, VAk
```cpp
//创建IMU预积分约束
EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);  
  
// 设置上一关键帧四个顶点（P、V、BG、BA）
// 当前帧两个顶点（P、V）
ei->setVertex(0, VPk);  
ei->setVertex(1, VVk);  
ei->setVertex(2, VGk);  
ei->setVertex(3, VAk);  
ei->setVertex(4, VP);  
ei->setVertex(5, VV);  

optimizer.addEdge(ei);
```
* [EdgeGyroRW](ORB_SLAM3_G2oType.md#EdgeGyroRW)
	* 设置vertex:
		* 上一帧：VGk
		* 当前帧：VG
	* 设置信息矩阵：[噪声更新](ORB_SLAM3_IMU预积分(理论推导).md#噪声更新)中的**矩阵C**
```cpp
EdgeGyroRW* egr = new EdgeGyroRW();  
  
//将上一关键帧的BG加入第三种边  
egr->setVertex(0,VGk);  
//将当前帧的BG加入第三种边  
egr->setVertex(1,VG);  
//C值在预积分阶段更新，range(9,12)对应陀螺仪偏置的协方差，最终cvInfoG值为inv(∑(GyroRW^2/freq))  
cv::Mat cvInfoG = pFrame->mpImuPreintegrated->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);  
Eigen::Matrix3d InfoG;  
for(int r=0;r<3;r++)  
	for(int c=0;c<3;c++)  
		InfoG(r,c)=cvInfoG.at<float>(r,c);  
  
//设置信息矩阵  
egr->setInformation(InfoG);  
//把第三种边加入优化器  
optimizer.addEdge(egr);
```
* [EdgeAccRW](ORB_SLAM3_G2oType.md#EdgeAccRW)
	*  设置vertex:
		* 上一帧：VGk
		* 当前帧：VG
	* 设置信息矩阵：[噪声更新](ORB_SLAM3_IMU预积分(理论推导).md#噪声更新)中的**矩阵C**
```cpp
EdgeAccRW* ear = new EdgeAccRW();  
//将上一关键帧的BA加入第四种边  
ear->setVertex(0,VAk);  
//将当前帧的BA加入第四种边  
ear->setVertex(1,VA);  
//C值在预积分阶段更新，range(12,15)对应加速度偏置的协方差，最终cvInfoG值为inv(∑(AccRW^2/freq))  
cv::Mat cvInfoA = pFrame->mpImuPreintegrated->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);  
Eigen::Matrix3d InfoA;  
for(int r=0;r<3;r++)  
	for(int c=0;c<3;c++)  
		InfoA(r,c)=cvInfoA.at<float>(r,c);  
//设置信息矩阵  
ear->setInformation(InfoA);  
//把第四种边加入优化器  
optimizer.addEdge(ear);
```
### 步骤四：优化策略
* 分4次优化，每次迭代10次
* 每次优化，评估每条重投影边的残差
	* 如果大于阈值，设置为`level = 1`，不再参与优化
	* 如果小于阈值，设置为`level = 0`
* 从第**3**次开始，不再使用鲁棒核函数`e->setRobustKernel(0)`
* **如果4次优化完后内点数目太少，恢复一些不太糟糕的点**
```cpp
    // 卡方检验值呈递减趋势，目的是让检验越来越苛刻
    float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
    float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};
    // 4次优化的迭代次数都为10
    int its[4] = {10, 10, 10, 10};

    // 坏点数
    int nBad = 0;
    // 单目坏点数
    int nBadMono = 0;
    // 双目坏点数
    int nBadStereo = 0;
    // 单目内点数
    int nInliersMono = 0;
    // 双目内点数
    int nInliersStereo = 0;
    // 内点数
    int nInliers = 0;
    bool bOut = false;

    // 进行4次优化
    for (size_t it = 0; it < 4; it++)
    {
        // 初始化优化器,这里的参数0代表只对level为0的边进行优化（不传参数默认也是0）
        optimizer.initializeOptimization(0);
        // 每次优化迭代十次
        optimizer.optimize(its[it]);

        // 每次优化都重新统计各类点的数目
        nBad = 0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers = 0;
        nInliersMono = 0;
        nInliersStereo = 0;

        // 使用1.5倍的chi2Mono作为“近点”的卡方检验值，意味着地图点越近，检验越宽松
        // 地图点如何定义为“近点”在下面的代码中有解释
        float chi2close = 1.5 * chi2Mono[it];

        // For monocular observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            // 如果这条误差边是来自于outlier
            if (pFrame->mvbOutlier[idx])
            {
                // 计算这条边上次优化后的误差
                e->computeError();
            }

            // 就是error*\Omega*error，表示了这条边考虑置信度以后的误差大小
            const float chi2 = e->chi2();

            // 当地图点在当前帧的深度值小于10时，该地图点属于close（近点）
            // mTrackDepth是在Frame.cc的isInFrustum函数中计算出来的
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

            // 判断某地图点为外点的条件有以下三种：
            // 1.该地图点不是近点并且误差大于卡方检验值chi2Mono[it]
            // 2.该地图点是近点并且误差大于卡方检验值chi2close
            // 3.深度不为正
            // 每次优化后，用更小的卡方检验值，原因是随着优化的进行，对划分为内点的信任程度越来越低
            if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive())
            {
                // 将该点设置为外点
                pFrame->mvbOutlier[idx] = true;
                // 外点不参与下一轮优化
                e->setLevel(1);
                // 单目坏点数+1
                nBadMono++;
            }
            else
            {
                // 将该点设置为内点（暂时）
                pFrame->mvbOutlier[idx] = false;
                // 内点继续参与下一轮优化
                e->setLevel(0);
                // 单目内点数+1
                nInliersMono++;
            }

            // 从第三次优化开始就不设置鲁棒核函数了，原因是经过两轮优化已经趋向准确值，不会有太大误差
            if (it == 2)
                e->setRobustKernel(0);
        }

        // For stereo observations
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1); // not included in next optimization
                nBadStereo++;
            }
            else
            {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        // 内点总数=单目内点数+双目内点数
        nInliers = nInliersMono + nInliersStereo;
        // 坏点数=单目坏点数+双目坏点数
        nBad = nBadMono + nBadStereo;

        if (optimizer.edges().size() < 10)
        {
            cout << "PIOLKF: NOT ENOUGH EDGES" << endl;
            break;
        }
    }

    // If not too much tracks, recover not too bad points
    // 9. 若4次优化后内点数小于30，尝试恢复一部分不那么糟糕的坏点
    if ((nInliers < 30) && !bRecInit)
    {
        // 重新从0开始统计坏点数
        nBad = 0;
        // 单目可容忍的卡方检验最大值（如果误差比这还大就不要挣扎了...）
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose *e1;
        EdgeStereoOnlyPose *e2;
        // 遍历所有单目特征点
        for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++)
        {
            const size_t idx = vnIndexEdgeMono[i];
            // 获取这些特征点对应的边
            e1 = vpEdgesMono[i];
            e1->computeError();
            // 判断误差值是否超过单目可容忍的卡方检验最大值，是的话就把这个点保下来
            if (e1->chi2() < chi2MonoOut)
                pFrame->mvbOutlier[idx] = false;
            else
                nBad++;
        }
        for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++)
        {
            const size_t idx = vnIndexEdgeStereo[i];
            e2 = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2() < chi2StereoOut)
                pFrame->mvbOutlier[idx] = false;
            else
                nBad++;
        }
    }
```
### 步骤五：恢复
```cpp
pFrame->SetImuPoseVelocity(Converter::toCvMat(VP->estimate().Rwb),Converter::toCvMat(VP->estimate().twb),Converter::toCvMat(VV->estimate()));  
Vector6d b;  
b << VG->estimate(), VA->estimate();  
//给当前帧设置优化后的bg，ba  
pFrame->mImuBias = IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]);
```
## 4.更新当前帧地图点实际被观测到次数
```cpp
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            //如果该地图点不为外点
            if(!mCurrentFrame.mvbOutlier[i])
            {
                // 真正找到该点的帧数mnFound 加 1
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                // 查看当前是否是在纯定位过程
                if(!mbOnlyTracking)
                {
                    // 如果该地图点被相机观测数目nObs大于0，匹配内点计数+1
                    // nObs： 被观测到的相机数目，单目+1，双目或RGB-D则+2
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    // 记录当前帧跟踪到的地图点数目，用于统计跟踪效果
                    mnMatchesInliers++;
            }
            // 如果这个地图点是外点,并且当前相机输入还是双目的时候,就删除这个点
            // 原因分析：因为双目本身可以左右互匹配，删掉无所谓
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }
```
## 5.判断跟踪是否成功
```cpp
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    // Step 5：根据跟踪匹配数目及重定位情况决定是否跟踪成功
    // 如果最近刚刚发生了重定位,那么至少成功匹配50个点才认为是成功跟踪
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    // RECENTLY_LOST状态下，至少成功跟踪10个才算成功
    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;

    // 单目IMU模式下做完初始化至少成功跟踪15个才算成功，没做初始化需要50个
    if (mSensor == System::IMU_MONOCULAR)
    {
        if((mnMatchesInliers<15 && mpAtlas->isImuInitialized())||(mnMatchesInliers<50 && !mpAtlas->isImuInitialized()))
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        //以上情况都不满足，只要跟踪的地图点大于30个就认为成功了
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }
```