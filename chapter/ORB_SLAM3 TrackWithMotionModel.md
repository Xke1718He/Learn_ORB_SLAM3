## TrackWithMotionModel
### 1.更新上一帧位姿
`Tracking::UpdateLastFrame()`的主要作用是**更新上一帧的位姿**和**添加一些临时的地图点**，为什么要更新上一帧的位姿，主要是在ORB_SLAM中优化的是**参考关键帧**的位姿，对于**普通帧**，虽然在开始设置了位姿，但是没有参与优化，因此在下一次跟踪时，需要用**优化后的参考关键帧**的位姿更新上一帧的位姿
```cpp
    // Update pose according to reference keyframe
    // Step 1：利用参考关键帧更新上一帧在世界坐标系下的位姿
    // 上一普通帧的参考关键帧，注意这里用的是参考关键帧（位姿准）而不是上上一帧的普通帧
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    // ref_keyframe 到 lastframe的位姿变换
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    // 将上一帧的世界坐标系下的位姿计算出来
    // l:last, r:reference, w:world
    // Tlw = Tlr*Trw 
    mLastFrame.SetPose(Tlr * pRef->GetPose());
```
* `mlRelativeFramePoses`存储**参考关键帧**`r`到**当前帧**`c`的位姿$T_{cr}$
$$
T_{cr} = T_{cw} \cdot T_{rw}^{-1}
$$

* 利用参考关键帧更新上一帧在世界坐标系下的**位姿**
$$
T_{lw} = T_{lr}\cdot T_{rw}
$$

对于**双目**或**rgbd**，为上一帧生成新的**临时地图点，主要是为了生成更多的匹配，让跟踪更好**
* 临时地图点：对于**上一帧**中具有有效深度值`z>0`的特征点，如果这个**特征点**在**上一帧**中**没有**对应的地图点，或者创建后**没有被观测到**，添加为临时地图点
* 临时地图点也不是越多越好，当满足下面两个条件停止添加：
	* 当前的点的深度已经超过了设定的深度阈值(**35倍基线**)，主要太远了不可靠
	* 临时地图点已超过`100`个点，说明距离比较远了，可能不准确，这里是从近的开始添加

```cpp
    // Step 2：对于双目或rgbd相机，为上一帧生成新的临时地图点
    // 注意这些地图点只是用来跟踪，不加入到地图中，跟踪完后会删除
    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // Step 2.1：得到上一帧中具有有效深度值的特征点（不一定是地图点）
    vector<pair<float,int> > vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for(int i=0; i<Nfeat;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            // vDepthIdx第一个元素是某个点的深度,第二个元素是对应的特征点id
            vDepthIdx.push_back(make_pair(z,i));
        }
    }
    // 如果上一帧中没有有效深度的点,那么就直接退出
    if(vDepthIdx.empty())
        return;

    // 按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // Step 2.2：从中找出不是地图点的部分
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
            // 地图点被创建后就没有被观测，认为不靠谱，也需要重新创建
            bCreateNew = true;

        if(bCreateNew)
        {
            // Step 2.3：需要创建的点，包装为地图点。只是为了提高双目和RGBD的跟踪成功率，并没有添加复杂属性，因为后面会扔掉
            // 反投影到世界坐标系中
            Eigen::Vector3f x3D;

            if(mLastFrame.Nleft == -1){
                mLastFrame.UnprojectStereo(i, x3D);
            }
            else{
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            // 加入上一帧的地图点中
            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
            mLastFrame.mvpMapPoints[i]=pNewMP;

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            // 因为从近到远排序，记录其中不需要创建地图点的个数
            nPoints++;
        }

        // Step 2.4：如果地图点质量不好，停止创建地图点
        // 停止新增临时地图点必须同时满足以下条件：
        // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
        // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;

    }
```
### 2.得到当前帧的初始位姿
如果IMU已初始化并且不需要`reset`时，使用`PredictStateIMU`来预测当前帧的状态，就不用通过**匀速模型**来得到了
#### PredictStateIMU
这里有两个变量控制着从哪预测
* `mbMapUpdated`：地图是否更新
* `mpLastKeyFrame`：上一关键帧存在

于是有两种情况：
* 如果地图更新了，且上一关键帧存在，则用关键帧来进行预测`mpImuPreintegratedFromLastKF`
* 如果地图未更新，则用上一帧来进行预测`mpImuPreintegratedFrame`


首先，根据`mpImuPreintegratedFromLastKF`或`mpImuPreintegratedFrame`得到$R_{b_{1}b_{2}}$、$v_{b_{1}b_{2}}$、$p_{b_{1}b_{2}}$

注意：这里可以参考前面的**IMU预积分的理论推导**中的预积分测量值更新(bias更新，一阶近似)
$$
\Delta \hat{\mathbf{R}}_{i j} \approx \Delta \overline{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)
$$
$$
\Delta \hat{\mathbf{V}}_{i j} \approx \Delta \overline{\mathbf{V}}_{i j}+\frac{\partial \Delta \overline{\mathbf{V}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{V}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}
$$

$$
\Delta \hat{\mathbf{p}}_{i j} \approx \Delta \overline{\mathbf{p}}_{i j}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}
$$
```cpp
Eigen::Matrix3f Preintegrated::GetDeltaRotation(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    // 计算偏置的变化量
    Eigen::Vector3f dbg;
    dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
    // 考虑偏置后，dR对偏置线性化的近似求解,邱笑晨《预积分总结与公式推导》P13～P14
    // Forster论文公式（44）yP17也有结果（但没有推导），后面两个函数GetDeltaPosition和GetDeltaPosition也是基于此推导的
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
}

/** 
 * @brief 根据新的偏置计算新的dP
 * @param b_ 新的偏置
 * @return dP
 */
Eigen::Vector3f Preintegrated::GetDeltaPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
    dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
    // 考虑偏置后，dP对偏置线性化的近似求解,邱笑晨《预积分总结与公式推导》P13，JPg和JPa在预积分处理中更新
    return dP + JPg * dbg + JPa * dba;
}

/** 
 * @brief 根据新的偏置计算新的dV
 * @param b_ 新的偏置
 * @return dV
 */
Eigen::Vector3f Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
    dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
    // 考虑偏置后，dV对偏置线性化的近似求解,邱笑晨《预积分总结与公式推导》P13，JPg和JPa在预积分处理中更新 
    return dV + JVg * dbg + JVa * dba;
}
```
然后，估计当前帧的$R_{wb_{2}}$、$t_{wb_{2}}$、$v_{wb_{2}}$

$$
\begin{align}
R_{j} & = \mathbf{R}_{i}\Delta R_{ij} \\
\mathbf{v}_{j} & = \mathbf{v}_{i}+\mathbf{g} \cdot \Delta t_{i j}+\mathbf{R}_{i}\Delta \mathbf{v}_{i j} \\
\mathbf{p}_{j} & = \mathbf{p}_{i} + \mathbf{v}_{i} \cdot \Delta t_{i j} + \frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}+ \mathbf{R}_{i}\Delta \mathbf{p}_{i j}  
\end{align}
$$
```cpp
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        // 计算当前帧在世界坐标系的位姿,原理都是用预积分的位姿（预积分的值不会变化）与上一帧的位姿（会迭代变化）进行更新 
        // 旋转 R_wb2 = R_wb1 * R_b1b2
        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        // 位移
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        // 速度 
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
```

* 设置当前帧的位姿$T_{cw}$和速度$V_{w}$

$$
\begin{align}
v_{w} & = v_{wb} \\
T_{cw} & = T_{cb}\cdot T_{bw}
\end{align}
$$
```cpp
// 设置当前帧的世界坐标系的相机位姿
mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

// 记录bias
mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;

/** 
 * @brief 赋值位姿与速度
 */
void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb)
{
    mVw = Vwb;
    mbHasVelocity = true;

    Sophus::SE3f Twb(Rwb, twb);
    Sophus::SE3f Tbw = Twb.inverse();

    mTcw = mImuCalib.mTcb * Tbw;

    UpdatePoseMatrices();
    mbIsSet = true;
    mbHasPose = true;
}
```
#### 匀速模型
当无法用`PredictStateIMU`预测当前帧的位姿与速度时，采用**匀速模型**

来看下这个速度$V$是什么：当跟踪成功或者刚刚跟丢，会更新该速度，该速度表示**上一帧到当前帧的变换**，其中$c$当前帧，$l$上一帧，$w$世界坐标系
$$
V = T_{cl} =T_{cw}\cdot T_{wl}
$$

```cpp
        if(bOK || mState==RECENTLY_LOST)
        {
            if(mLastFrame.isSet() && mCurrentFrame.isSet())
            {
                Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                mVelocity = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity = true;
            }
            else {
                // 否则没有速度
                mbVelocity = false;
            }
          }
```
首先，假设两帧之间的变换近似，根据速度设置当前帧的初始位姿
$$
T_{cw} = T_{cl}\cdot T_{lw}
$$
然后，基于**投影的匹配搜索**`SearchByProjection`获得上一帧与当前帧的匹配关系，其步骤：
* 构建旋转直方图，用于检测旋转一致性
```cpp
        // Rotation Histogram (to check rotation consistency)
        // Step 1 建立旋转直方图，用于检测旋转一致性
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;
```
* 计算当前帧和前一帧的平移$t_{lc}$，判断相机是前进还是后退（近大远小）（尺度越大，图像越小）
	* 前进：$z > b$，物体在当前帧的图像上**变大**，因此对于上一帧的特征点，需要在当前帧**更高**的尺度上搜索
	* 后退：$z < -b$，物体在当前帧的图像上**变小**，因此对于上一帧的特征点，需要在当前帧**更低**的尺度上搜索
```cpp
        // Step 2 计算当前帧和前一帧的平移向量
        //当前帧的相机位姿
        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        const Eigen::Vector3f twc = Tcw.inverse().translation();

        const Sophus::SE3f Tlw = LastFrame.GetPose();
        const Eigen::Vector3f tlc = Tlw * twc; 

        // 判断前进还是后退
        const bool bForward = tlc(2)>CurrentFrame.mb && !bMono;     // 非单目情况，如果Z大于基线，则表示相机明显前进
        const bool bBackward = -tlc(2)>CurrentFrame.mb && !bMono;   // 非单目情况，如果-Z小于基线，则表示相机明显后退
```
* 对于前一帧的每一个地图点，通过相机投影模型，投影到当前帧
```cpp
         // 对上一帧有效的MapPoints投影到当前帧坐标系
         Eigen::Vector3f x3Dw = pMP->GetWorldPos();
         Eigen::Vector3f x3Dc = Tcw * x3Dw;

         const float xc = x3Dc(0);
         const float yc = x3Dc(1);
         const float invzc = 1.0/x3Dc(2);

         if(invzc<0)
             continue;

         // 投影到当前帧中
         Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/10bf7c4ba9c44055a9accc0e6fb45565.png)
* 根据相机的前进后退方向来判断搜索尺度范围
	* 前进：$z > b$，物体在当前帧的图像上**变大**，因此对于上一帧的特征点，需要在当前帧**更高**的尺度上搜索
	* 后退：$z < -b$，物体在当前帧的图像上**变小**，因此对于上一帧的特征点，需要在当前帧**更低**的尺度上搜索
![在这里插入图片描述](https://img-blog.csdnimg.cn/be2af519b6c449a09446d30c3385d268.png)


```cpp
        // 认为投影前后地图点的尺度信息不变
        int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
                                                                        : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

        // Search in a window. Size depends on scale
        // 单目：th = 7，双目：th = 15
        float radius = th*CurrentFrame.mvScaleFactors[nLastOctave]; // 尺度越大，搜索范围越大

        // 记录候选匹配点的id
        vector<size_t> vIndices2;

        // Step 4 根据相机的前后前进方向来判断搜索尺度范围。
        // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
        // 当相机前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
        // 当相机后退时，圆点的面积减小，在某个尺度m下它是一个特征点，由于面积减小，则需要在更低的尺度下才能检测出来
        if(bForward)  // 前进,则上一帧兴趣点在所在的尺度nLastOctave<=nCurOctave
            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave);
        else if(bBackward)  // 后退,则上一帧兴趣点在所在的尺度0<=nCurOctave<=nLastOctave
            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, 0, nLastOctave);
        else  // 在[nLastOctave-1, nLastOctave+1]中搜索
            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave-1, nLastOctave+1);

        if(vIndices2.empty())
            continue;
```
* 遍历候选匹配点，寻找距离最小的最佳匹配点
	* 这里就简单选了个最佳匹配点，其他的像剔除重复匹配，最佳和次佳比都没做
```cpp
                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    // Step 5 遍历候选匹配点，寻找距离最小的最佳匹配点 
                    for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                    {
                        const size_t i2 = *vit;

                        // 如果该特征点已经有对应的MapPoint了,则退出该次循环
                        if(CurrentFrame.mvpMapPoints[i2])
                            if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                                continue;

                        if(CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2]>0)
                        {
                            // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                            const float ur = uv(0) - CurrentFrame.mbf*invzc;
                            const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                            if(er>radius)
                                continue;
                        }

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP,d);

                        if(dist<bestDist)
                        {
                            bestDist=dist;
                            bestIdx2=i2;
                        }
                    }
```
* 计算匹配点对的**旋转角度差**所在的直方图
```cpp
                    // 最佳匹配距离要小于设定阈值
                    if(bestDist<=TH_HIGH)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                        nmatches++;

                        // Step 6 计算匹配点旋转角度差所在的直方图
                        if(mbCheckOrientation)
                        {
                            cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                                        : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                                                : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                            cv::KeyPoint kpCF = (CurrentFrame.Nleft == -1) ? CurrentFrame.mvKeysUn[bestIdx2]
                                                                        : (bestIdx2 < CurrentFrame.Nleft) ? CurrentFrame.mvKeys[bestIdx2]
                                                                                                            : CurrentFrame.mvKeysRight[bestIdx2 - CurrentFrame.Nleft];
                            float rot = kpLF.angle-kpCF.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }
```
* 进行旋转一致检测，剔除不一致的匹配
![在这里插入图片描述](https://img-blog.csdnimg.cn/69ca90e110524f1689d938135b2aeeb9.png)
```cpp
        //Apply rotation consistency
        //  Step 7 进行旋转一致检测，剔除不一致的匹配
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                // 对于数量不是前3个的点对，剔除
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                        nmatches--;
                    }
                }
            }
        }
```


### 3.优化
得到上一帧与当前帧的匹配关系后，利用`3D-2D`投影关系优化当前帧位姿`PoseOptimization`
#### PoseOptimization
`PoseOptimization`主要的作用是利用**重投影**优化**单帧的位姿**，主要用在`Tracking`的几种跟踪模式`TrackWithMotionModel`、`TrackReferenceKeyFrame`、 `TrackLocalMap`、`Relocalization`中
![](https://img-blog.csdnimg.cn/42ae5eb09cab453abc35493835d222f3.png)
  
##### 输入  
|  优化变量        |              |  观测            |
|:-------------|:-------------|:---------------|
| 帧的Pose       | 帧的MapPoint   | 帧的KeyPoint     |     
##### 初始化  
```cpp  
	//创建优化器  
    g2o::SparseOptimizer optimizer;  
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

	//创建线性求解器
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	//创建块求解器 6 位姿 3 地图点
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	//设置优化算法
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
```
##### 设置vertex
* VertexSE3Expmap
	* 设置**估计值**：$T_{cw}$
	* 设置Id
	* 是否固定：**False**
```cpp
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pFrame->GetPose();
    //需要将Tcw转换为SE3Quat
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);
```
##### VertexSE3Expmap
| 优化变量     | 类型        | 维度 |
|:---------|:----------|:---|
| $T_{cw}$ | `SE3Quat` |  6 |  
* `VertexSE3Expmap`：SE3类型顶点在内部用变换矩阵参数化，在外部用指数映射参数化
* `SE3Quat`用**四元数**表示旋转，在更新时将**6维的前3维**通过李群李代数进行指数映射为旋转矩阵，然后再转换为四元数，内部操作采用四元数
* 更新：
$$
 \tilde{T}_{cw} = \mathbf{Exp}{\delta\vec{\phi}}\cdot T_{cw}
$$
```cpp
// 6维，类型SE3Quat
class  VertexSE3Expmap : public BaseVertex<6, SE3Quat>{  
public:  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  
  VertexSE3Expmap();  
  
  bool read(std::istream& is);  
  
  bool write(std::ostream& os) const;  
  
  virtual void setToOriginImpl() {  
    _estimate = SE3Quat();  
  }  
  
  virtual void oplusImpl(const double* update_)  { 
	  //获得delta 
    Eigen::Map<const Vector6d> update(update_);  
    // 更新，指数映射，设置估计值
    setEstimate(SE3Quat::exp(update)*estimate());  
  }  
};
```
##### 设置edge
对于每一对**地图点-特征点**添加重投影残差边：
* [EdgeSE3ProjectXYZOnlyPose](ORB_SLAM3_G2oType.md#EdgeSE3ProjectXYZOnlyPose)
	* 设置vertex：`g2o::VertexSE3Expmap`
	* 设置观测**obs**：`keyPoint`
	* 设置**信息矩阵**
	* 设置**鲁棒核函数**：huber核
	* 设置huber核的的δ
	* 设置**相机内参**
	* 设置**地图点**
```cpp
	Eigen::Matrix<double,2,1> obs;  
	const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];  
	obs << kpUn.pt.x, kpUn.pt.y;

	// 新建节点，只优化位姿  
   ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

	//设置vertex和观测
   e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));  
   e->setMeasurement(obs);  

	//设置信息矩阵
	const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];  
	e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

	//设置huber核函数
	g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;  
	e->setRobustKernel(rk);  
	rk->setDelta(deltaMono);

	//设置相机内参
	e->pCamera = pFrame->mpCamera;  
	//设置地图点
	cv::Mat Xw = pMP->GetWorldPos();  
	e->Xw[0] = Xw.at<float>(0);  
	e->Xw[1] = Xw.at<float>(1);  
	e->Xw[2] = Xw.at<float>(2);
  
   optimizer.addEdge(e);
```
##### EdgeSE3ProjectXYZOnlyPose
* 属性：**一元边**
* 观测：$p=\left(u,v\right)$
* 优化变量：$T_{cw}$
* 残差：$err = p-\pi \left(T_{cw} \cdot P_{w}\right)$

```cpp
// 2 观测的维度
// Eigen::Vector2d 观测的类型
// g2o::VertexSE3Expmap vertex的类型
class  EdgeSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{  
public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  
    EdgeSE3ProjectXYZOnlyPose(){}  
  
    bool read(std::istream& is);  
  
    bool write(std::ostream& os) const;  
  
    void computeError()  {  
	    //获取顶点
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);  
        //获取观测
        Eigen::Vector2d obs(_measurement);  
        //计算残差
        _error = obs-pCamera->project(v1->estimate().map(Xw));  
    }  
  
    bool isDepthPositive() {  
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);  
        return (v1->estimate().map(Xw))(2)>0.0;  
    }  
  
  
    virtual void linearizeOplus();  
  
    Eigen::Vector3d Xw;  //地图点
    GeometricCamera* pCamera;  //相机模型
};
```

* jacobian
$$
\frac{\partial \boldsymbol{e}}{\partial \delta \boldsymbol{\xi}}=\frac{\partial \boldsymbol{e}}{\partial \boldsymbol{P}_{C}} \frac{\partial \boldsymbol{P}_{C}}{\partial \delta \boldsymbol{\xi}}
$$
其中：
$$
\frac{\partial \boldsymbol{e}}{\partial \boldsymbol{P}^{C}}=-\left(\begin{array}{lll}
\frac{\partial u_{\mathrm{cal}}}{\partial X^{C}} & \frac{\partial u_{\mathrm{cal}}}{\partial Y^{C}} & \frac{\partial u_{\mathrm{cal}}}{\partial Z^{C}} \\
\frac{\partial v_{\mathrm{cal}}}{\partial X^{C}} & \frac{\partial v_{\mathrm{cal}}}{\partial Y^{C}} & \frac{\partial v_{\mathrm{cal}}}{\partial Z^{C}}
\end{array}\right)=-\left(\begin{array}{ccc}
\frac{f_{z}}{Z^{C}} & 0 & -\frac{f_{x} X^{C}}{\left(Z^{C}\right)^{2}} \\
0 & \frac{f_{y}}{Z^{C}} & -\frac{f_{y} Y^{C}}{\left(Z^{C}\right)^{2}}
\end{array}\right)
$$

$$
\frac{\partial \boldsymbol{P}^{C}}{\partial \delta \boldsymbol{\xi}}=\frac{\partial\left(\boldsymbol{T} \boldsymbol{P}^{W}\right)}{\partial \delta \boldsymbol{\xi}}=\left(\boldsymbol{T} \boldsymbol{P}^{W}\right)^{\odot}
=\begin{bmatrix}
 I &\left(\boldsymbol{P}^{W}\right)^{\wedge}
\end{bmatrix}
$$
```cpp
void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {  
    g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);  
    Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);  
  
    double x = xyz_trans[0];  
    double y = xyz_trans[1];  
    double z = xyz_trans[2];  
  
    Eigen::Matrix<double,3,6> SE3deriv;  
    SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,  
                 -z , 0.f, x, 0.f, 1.f, 0.f,  
                 y ,  -x , 0.f, 0.f, 0.f, 1.f;  
  
    _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * SE3deriv;  
}
```
##### 优化策略
* 分4次优化，每次迭代10次
* 每次优化，评估每条重投影边的残差
	* 如果大于阈值，设置为`level = 1`，不再参与优化
	* 如果小于阈值，设置为`level = 0`
* 从第**3**次开始，不再使用鲁棒核函数`e->setRobustKernel(0)`
##### 恢复
```cpp
g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));  
g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();  
cv::Mat pose = Converter::toCvMat(SE3quat_recov);  
pFrame->SetPose(pose);
```
### 4.剔除当前帧中地图点中的外点
```cpp
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                // 如果优化后判断某个地图点是外点，清除它的所有关系
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                // 累加成功匹配到的地图点数目
                nmatchesMap++;
        }
    }
```