## 单目初始化
`单目初始化`也就是通过前后两帧图像(`mInitialFrame`与`mCurrentFrame`)进行特征匹配，得到他们之间的**匹配关系**`mvIniMatches`后，通过**H**或者**F**恢复两帧之间的运动，并通过**三角化**生成地图点
![在这里插入图片描述](https://img-blog.csdnimg.cn/b2afd939b78140589b3c98d814e202c7.png)
### 1.接口
```cpp
void Tracking::MonocularInitialization()
```
### 2.步骤
`单目初始化`的大致步骤如下：
1. 如果**当前帧**`mCurrentFrame`与**初始帧**`mInitialFrame`满足如下条件，并创建`当前帧`和`来自上一关键帧`的**IMU预积分**:
	* **当前帧**`mCurrentFrame`的特征点数大于100
	* **初始帧**`mInitialFrame`的特征点数大于100
	* 两帧之间的时间戳间隔小于1(**IMU模式下**)
	* `mCurrentFrame.mpImuPreintegrated`：当前帧的**IMU预积分**
	* `mpImuPreintegratedFromLastKF`：来自上一关键帧的**IMU预积分**
2. 在`mInitialFrame`与`mCurrentFrame`中通过特征匹配(`SearchForInitialization`)得到对应的**匹配关系**`mvIniMatches`
3. 通过**H模型**或**F模型**进行单目初始化，得到两帧间相对运动`ReconstructWithTwoViews`、初始MapPoints
4. 删除那些无法进行三角化的匹配点
5. 通过**三角化**生成3D点`CreateInitialMapMonocular`，进而生成地图点MapPoints


**接下来，详细介绍单目初始化中的每一部分**
### 3.SearchForInitialization
`SearchForInitialization`主要的作用是寻找**初始帧F1**与**当前帧F2**之间的匹配关系`vnMatches12`，其输入参数如下：
|  参数                                                                                                                                                                                                |  描述            |
|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:---------------|
| F1                                                                                                                                                                                               | 初始帧            |
| F2                                                                                                                                                                                                 | 当前帧            |
| vbPrevMatched                                                                                                                                                                                      | 初始帧中特征点的坐标     |
| vnMatches12                                                                                                                                                                                        | 匹配关系           |
| windowSize                                                                                                                                                                                         | 搜索窗口大小         |  

其组件包括：搜索窗口、重复匹配过滤、最佳描述子距离、最优与次优比值、旋转直方图
#### 1.旋转直方图的构建
```cpp
        // Step 1 构建旋转直方图，HISTO_LENGTH = 30
        vector<int> rotHist[HISTO_LENGTH];
        // 每个bin里预分配500个，因为使用的是vector不够的话可以自动扩展容量
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;
```
#### 2.搜索候选匹配点
遍历`F1`中的所有特征点，在`F2`上寻找候选匹配点
![在这里插入图片描述](https://img-blog.csdnimg.cn/470f6f5fa6cd49acb5f26a2c7bae9d8e.png)

```cpp
        // 遍历帧1中的所有特征点
        for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
        {
            cv::KeyPoint kp1 = F1.mvKeysUn[i1];
            int level1 = kp1.octave;
            // 只使用原始图像上提取的特征点
            if(level1>0)
                continue;

            // Step 2 在半径窗口内搜索当前帧F2中所有的候选匹配特征点 
            // vbPrevMatched 输入的是参考帧 F1的特征点
            // windowSize = 100，输入最大最小金字塔层级 均为0
            vector<size_t> vIndices2 = 	F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);
```
#### 3.通过描述子的距离搜索候选匹配点中的最优与次优
 * 注意：`vMatchedDistance`
	* 描述：**vMatchedDistance**的size为**F2**的特征点数目，**vMatchedDistance**用于存储与**F2**的第**index**特征点匹配**最佳**的**描述子距离**
	* 作用：当**vMatchedDistance**的第**index**位置**非INT_MAX**，那么说明**F1**中已经有特征点和其匹配上了，**F1**中其他特征点想和**F2**中第**index**特征点再匹配上，就必须更好，小于此距离。
	* 例子：比如`F1`中的特征点`1`与`F2`中的特征点`2`已匹配上，那么`F1`中特征点`2`搜到的候选匹配点包括`0, 1, 2, 3`，分别计算对应的描述子距离，对于`F2`中的特征点`2`，用`vMatchedDistance`对其进行过滤(`5 > 4`)
![在这里插入图片描述](https://img-blog.csdnimg.cn/722f3ae0afad4a099b446b96f00d5e38.png)


```cpp
            // Step 3 遍历搜索搜索窗口中的所有潜在的匹配候选点，找到最优的和次优的
            for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
            {
                size_t i2 = *vit;
                // 取出候选特征点对应的描述子
                cv::Mat d2 = F2.mDescriptors.row(i2);
                // 计算两个特征点描述子距离
                int dist = DescriptorDistance(d1,d2);

                if(vMatchedDistance[i2]<=dist)
                    continue;
                // 如果当前匹配距离更小，更新最佳次佳距离
                if(dist<bestDist)
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestIdx2=i2;
                }
                else if(dist<bestDist2)
                {
                    bestDist2=dist;
                }
            }
```
#### 4.条件筛选
* 最佳描述子距离小于阈值
* 最佳距离比次佳距离要小于设定的比例
* 删除重复匹配
```cpp
            // Step 4 对最优次优结果进行检查，满足阈值、最优/次优比例，删除重复匹配
            // 即使算出了最佳描述子匹配距离，也不一定保证配对成功。要小于设定阈值
            if(bestDist<=TH_LOW)
            {
                // 最佳距离比次佳距离要小于设定的比例，这样特征点辨识度更高
                if(bestDist<(float)bestDist2*mfNNratio)
                {
                    // 如果找到的候选特征点对应F1中特征点已经匹配过了，说明发生了重复匹配，将原来的匹配也删掉
                    if(vnMatches21[bestIdx2]>=0)
                    {
                        vnMatches12[vnMatches21[bestIdx2]]=-1;
                        nmatches--;
                    }
                    // 次优的匹配关系，双向建立
                    // vnMatches12保存参考帧F1和F2匹配关系，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
                    vnMatches12[i1]=bestIdx2;
                    vnMatches21[bestIdx2]=i1;
                    vMatchedDistance[bestIdx2]=bestDist;
                    nmatches++;

                    // Step 5 计算匹配点旋转角度差所在的直方图
                    if(mbCheckOrientation)
                    {
                        // 计算匹配特征点的角度差，这里单位是角度°，不是弧度
                        float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        // 前面factor = HISTO_LENGTH/360.0f 
                        // bin = rot / 360.of * HISTO_LENGTH 表示当前rot被分配在第几个直方图bin  
                        int bin = round(rot*factor);
                        // 如果bin 满了又是一个轮回
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(i1);
                    }
                }
            }
```
* vnMatches21的作用
	* 描述：**vnMatches21**的size为**F2**的特征点数目，**vnMatches21**用于存储**F1**中与**F2**的第**index**特征点匹配**最佳**的**索引**
	* 作用：当**vMatches21**的第**index**位置**非-1**，那么说明**F1**中已经有特征点**m**和其匹配上了，如果还有**F1**中其他特征点和**F2**中第**index**特征点匹配上，那么此为重复匹配，上一次的匹配结果**m**就不能要(vnMatches12[m] = -1)，最后将这次的匹配结果保存到**vnMatches21**和**vMatchedDistance**的index位置。
	* `F1`中的第`0`个特征点和`F2`中的第`3`个特征点已经匹配，现在F1中的第`3`个特征点也和F2中的第`3`个特征点匹配上了，那么需要将F2中的第`3`个特征点的最佳匹配**更新**为F1中的第`3`个特征点，且在最终结果`vnMathes12`中将F1中第`0`个特征点的匹配置为无效(`-1`)，**最后更新**F2中第3个特征点的**最佳距离**
![在这里插入图片描述](https://img-blog.csdnimg.cn/ec30ff73a74841deb36c3e33a0754fc7.png)
#### 5.旋转直方图
统计旋转直方图，选取前三个方向，其余全部剔除，因为匹配的方向具有一致性
![在这里插入图片描述](https://img-blog.csdnimg.cn/f8693a5caa1a4c5fa2939ffcb166c762.png)
```cpp
        // Step 6 筛除旋转直方图中“非主流”部分
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;
            // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                // 剔除掉不在前三的匹配对，因为他们不符合“主流旋转方向”    
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    int idx1 = rotHist[i][j];
                    if(vnMatches12[idx1]>=0)
                    {
                        vnMatches12[idx1]=-1;
                        nmatches--;
                    }
                }
            }

        }
```
#### 6.更新匹配
```cpp
        // Step 7 将最后通过筛选的匹配好的特征点保存到vbPrevMatched
        for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
            if(vnMatches12[i1]>=0)
                vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;
```
### 4.Reconstruct
在通过**单应性矩阵H**或者**基础矩阵F**进行运动恢复中，采用了**Ransac**方法来估计H或者F
```cpp
        // Step 5 通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated))
```
#### 1.RANSAC
RANSAC是"RANdom SAmple Consensus"（随机采样一致）的缩写。该算法主要通过一组**包含“外点”（即错误点）的数据**通过**迭代**的方式估计**数学模型的参数**，特别注意：内点的数量需要大于外点，不然估计出来的模型就是满足外点的
```cpp
//存放RANSAC每次迭代随机选取的8个索引
mvSets = vector<vector<size_t>>(mMaxIterations, vector<size_t>(8, 0));
DUtils::Random::SeedRandOnce(0);
// 2. 先遍历把200次先取好
for (int it = 0; it < mMaxIterations; it++)
{
	vAvailableIndices = vAllIndices;
	// Select a minimum set
	for (size_t j = 0; j < 8; j++)
	{
		int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
		int idx = vAvailableIndices[randi]; // 这句不多余，防止重复选择
		mvSets[it][j] = idx;
		// 保证选不到同一个点，这么做的话可以删去vAvailableIndices已选点的索引
		vAvailableIndices[randi] = vAvailableIndices.back();
		vAvailableIndices.pop_back();
	}
}
```
 vAvailableIndices的作用:
* `vAvailableIndices`的`index`为**随机抽样**的值；
* `vAvailableIndices`的`value`为**匹配关系**的索引值；
* 为了防止随机抽样时索引重复，当随机抽样得到`vAvailableIndices`的`index`，保存该`index`对应的`value`到`mvsets`中，然后用末尾的索引代替`vAvailableIndices[randi]`的值，这也是为了保证末尾的索引在下次抽样中能有机会抽到，最后将末尾删除掉，那么`vAvailableIndices`的`size`减小，下次随机抽样就在剩下的`index`中抽样。

![](https://img-blog.csdnimg.cn/b996f6bbc7094cd0b458a31d0b59217d.png)

#### 2.归一化
* 问题：特征点中心偏离中心点以及分布不均匀
* 作用：
	* 特征点中心在图像中心
	* 特征点各个方向平均值相等，即到图像中心点的距离为$\sqrt{2}$

$x$和$y$为特征点的原始坐标，$\hat{x}$和$\hat {y}$为特征点的归一化坐标

$$
\begin{align}
\underbrace{\left (x-\bar{x}\right )} \cdot\underbrace{\frac{N}{\sum_{i}^{N}  \left |x_{i}-\bar{x}\right |   } }_{s_x} \\
\underbrace{\left (y-\bar{y}\right )} \cdot\underbrace{\frac{N}{\sum_{i}^{N}  \sqrt{y_{i}-\bar{y} } } }_{s_y} 
\end{align}\Rightarrow \underbrace{\begin{bmatrix}
  \hat{x} \\
  \hat{y}\\
1
\end{bmatrix}=\begin{bmatrix}
 s_x & 0&-meanX\cdot s_x  \\
 0 & s_y & -meanY\cdot s_y\\
 0 & 0 & 1
\end{bmatrix}}_{T} \begin{bmatrix}
x \\
y \\
1
\end{bmatrix}
$$

```cpp
/** 
 * @brief 像素坐标标准化，计算点集的横纵均值，与均值偏差的均值。最后返回的是变化矩阵T 直接乘以像素坐标的齐次向量即可获得去中心去均值后的特征点坐标
 * @param vKeys 特征点
 * @param vNormalizedPoints 去中心去均值后的特征点坐标
 * @param T  变化矩阵
 */
void TwoViewReconstruction::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, Eigen::Matrix3f &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for (int i = 0; i < N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    // 1. 求均值
    meanX = meanX / N;
    meanY = meanY / N;

    float meanDevX = 0;
    float meanDevY = 0;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    // 2. 确定新原点后计算与新原点的距离均值
    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    // 3. 去均值化
    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    // 4. 计算变化矩阵
    T.setZero();
    T(0, 0) = sX;
    T(1, 1) = sY;
    T(0, 2) = -meanX * sX;
    T(1, 2) = -meanY * sY;
    T(2, 2) = 1.f;
}
```
#### 3.FindFundamental
* 基础矩阵计算
由`极线约束`可知：
$$
p_{2}^{T} F p_{1}=0
$$
假设一对匹配的像点$p_1 = \left[u_{1}, v_{1}, 1 \right]^T$, $p_2 = \left[u_{2}, v_{2}, 1 \right]^T$，带入上式中，得到：
$$
\left[u_{2}, v_{2}, 1\right]\left[\begin{array}{ccc}
f_{1} & f_{2} & f_{3} \\
f_{4} & f_{5} & f_{6} \\
f_{7} & f_{8} & f_{9}
\end{array}\right]\left[\begin{array}{c}
u_{1} \\
v_{1} \\
1
\end{array}\right]=0
$$
将其展开得到：
$$
\left[u_{2} u_{1}, u_{2} v_{1}, u_{2}, v_{2} u_{1}, v_{2} v_{1}, v_{2}, u_{1}, v_{1}, 1\right] \cdot \boldsymbol{f}=0
$$
其中，`f`为$f=\left[f_{1}, f_{2}, f_{3}, f_{4}, f_{5}, f_{6}, f_{7}, f_{8}, f_{9}\right]$
对于其他的点对使用同样的方式，得到一个线程组：
$$
\left(\begin{array}{ccccccccc}
u_{2}^{1} u_{1}^{1} & u_{2}^{1} v_{1}^{1} & u_{2}^{1} & v_{2}^{1} u_{1}^{1} & v_{2}^{1} v_{1}^{1} & v_{2}^{1} & u_{1}^{1} & v_{1}^{1} & 1 \\
u_{2}^{2} u_{1}^{2} & u_{2}^{2} v_{1}^{2} & u_{2}^{2} & v_{2}^{2} u_{1}^{2} & v_{2}^{2} v_{1}^{2} & v_{2}^{2} & u_{1}^{2} & v_{1}^{2} & 1 \\
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \\
u_{2}^{8} u_{1}^{8} & u_{2}^{8} v_{1}^{8} & u_{2}^{8} & v_{2}^{8} u_{1}^{8} & v_{2}^{8} v_{1}^{8} & v_{2}^{8} & u_{1}^{8} & v_{1}^{8} & 1
\end{array}\right)\left(\begin{array}{c}
f_{1} \\
f_{2} \\
f_{3} \\
f_{4} \\
f_{5} \\
f_{6} \\
f_{7} \\
f_{8} \\
f_{9}
\end{array}\right)=0
$$
上述方程组可以转换为：求解f使得$\left \| Af \right \|最小且$$\left \| f \right \|=1$
对$A$进行`SVD`分解($A=U\Sigma V$)，得到：
$$
f=V_{9}
$$
基础矩阵F的另外一个重要性质：秩为2
$$
F = \begin{bmatrix}
f_1  & f_2 & f_3\\
f_4  & f_5 & f_6\\
 f_7 & f_8 & f_9
\end{bmatrix}
$$
则对F进一步的SVD分解，并设最小特征值$v_{3}$为0，最后得到F：
$$
F=U_2\Sigma V_2^T =U_2\left[\begin{array}{ccc}
v_{1} & 0 & 0 \\
0 & v_{2} & 0 \\
0 & 0 & 0
\end{array}\right] V_2^T
$$
归一化前的基础矩阵
$$
{x_{2}^{\prime}}^{T}Fx_{1}^{\prime}  = x_{2}^{T}\underbrace{T_{2}^{T}FT_{1}}_{F^{\prime}} x_{1}  = 0
$$
```cpp
Eigen::Matrix3f TwoViewReconstruction::ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    Eigen::MatrixXf A(N, 9);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A(i, 0) = u2 * u1;
        A(i, 1) = u2 * v1;
        A(i, 2) = u2;
        A(i, 3) = v2 * u1;
        A(i, 4) = v2 * v1;
        A(i, 5) = v2;
        A(i, 6) = u1;
        A(i, 7) = v1;
        A(i, 8) = 1;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Fpre(svd.matrixV().col(8).data());

    Eigen::JacobiSVD<Eigen::Matrix3f> svd2(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector3f w = svd2.singularValues();
    // 这里注意计算完要强制让第三个奇异值为0
    w(2) = 0;

    return svd2.matrixU() * Eigen::DiagonalMatrix<float, 3>(w) * svd2.matrixV().transpose();
}
```
* 双向校验：
根据基础矩阵$F_{21}$，进行**双向校验**：
极线$l_2$
$$
l_2 = F_{21}\cdot x_{1} = \left(a_2, b_2, c_2\right)
$$
$x_2$到$l_2$的距离
$$
e_2 = \frac{\left(a_{2}x_2+b_{2}y_2+c_2\right)^2 }{a_2^2+b_2^2} \cdot \frac{1}{\sigma^{2}} < th
$$
极线$l_1$
$$
l_1 = x_{2}^T\cdot F_{21} = \left(a_1, b_1, c_1\right)
$$
$x_1$到$l_1$的距离
$$
e_1 = \frac{\left(a_{1}x_1+b_{1}y_1+c_1\right)^2 }{a_1^2+b_1^2} \cdot \frac{1}{\sigma^{2}} < th
$$
```cpp
/** 
 * @brief 检查结果
 * @param F21 顾名思义
 * @param vbMatchesInliers 匹配是否合法，大小为mvMatches12
 * @param sigma 默认为1
 */
float TwoViewReconstruction::CheckFundamental(const Eigen::Matrix3f &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21(0, 0);
    const float f12 = F21(0, 1);
    const float f13 = F21(0, 2);
    const float f21 = F21(1, 0);
    const float f22 = F21(1, 1);
    const float f23 = F21(1, 2);
    const float f31 = F21(2, 0);
    const float f32 = F21(2, 1);
    const float f33 = F21(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;

    // 基于卡方检验计算出的阈值 自由度为1的卡方分布，显著性水平为0.05，对应的临界阈值
    const float th = 3.841;
    // 基于卡方检验计算出的阈值 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0 / (sigma * sigma);

    for (int i = 0; i < N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)
        // 计算 img1 上的点在 img2 上投影得到的极线 l2 = F21 * p1 = (a2,b2,c2)
        const float a2 = f11 * u1 + f12 * v1 + f13;
        const float b2 = f21 * u1 + f22 * v1 + f23;
        const float c2 = f31 * u1 + f32 * v1 + f33;

        // 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num2 = a2 * u2 + b2 * v2 + c2;

        const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        // 自由度为1是因为这里的计算是点到线的距离，判定分数自由度为2的原因可能是为了与H矩阵持平
        if (chiSquare1 > th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)
        // 与上面相同只不过反过来了
        const float a1 = f11 * u2 + f21 * v2 + f31;
        const float b1 = f12 * u2 + f22 * v2 + f32;
        const float c1 = f13 * u2 + f23 * v2 + f33;

        const float num1 = a1 * u1 + b1 * v1 + c1;

        const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if (bIn)
            vbMatchesInliers[i] = true;
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}
```
#### 4.FindHomography
* 单应性矩阵
由`单应性变换`得到：
$$
p_{2}=\hat{H} \cdot p_{1}
$$
假设一对匹配的像点$p_1 = \left[u_{1}, v_{1}, 1 \right]^T$, $p_2 = \left[u_{2}, v_{2}, 1 \right]^T$，带入上式中，得到：
$$
\left[\begin{array}{c}
u_{2} \\
v_{2} \\
1
\end{array}\right]=\left[\begin{array}{lll}
h_{1} & h_{2} & h_{3} \\
h_{4} & h_{5} & h_{6} \\
h_{7} & h_{8} & h_{9}
\end{array}\right]\left[\begin{array}{l}
u_{1} \\
v_{1} \\
1
\end{array}\right]
$$
将其展开得到：
$$
\left\{\begin{array}{c}
u_{2}=h_{1} u_{1}+h_{2} v_{1}+h_{3} \\
v_{2}=h_{4} u_{1}+h_{5} v_{1}+h_{6} \\
1=h_{7} u_{1}+h_{8} v_{1}+h_{9}
\end{array}\right.
$$
得到方程组：
$$
\begin{array}{l}
-h_{4}u_{1}-h_{5}v_{1}-h_{6}+h_{7}u_{1}v_{2}+h_{8}v_{1}v_{2} + v_{2}=0 \\
h_{1}u_{1}+h_{2}v_{1}+h_{3}-h_{7}u_{1}u_{2}-h_{8}u_{2}v_{1}-u_{2}=0
\end{array}
$$
由**四对点对**得到$Ah=0$形式：
$$
\underbrace{\left(\begin{array}{cccccccc}
0 & 0 & 0 & -u_{1}^{1} & -v_{1}^{1} & -1 & u_{1}^{1} v_{2}^{1} & v_{1}^{1} v_{2}^{1} &v_{2}^{1} \\
u_{1}^{1} & v_{1}^{1} & 1 & 0 & 0 & 0 & -u_{1}^{1} u_{2}^{1} & -v_{1}^{1} u_{2}^{1} &-u_{2}^{1} \\
0 & 0 & 0 & -u_{1}^{2} & -v_{1}^{2} & -1 & u_{1}^{2} v_{2}^{2} & v_{1}^{2} v_{2}^{2} &v_{2}^{2} \\
u_{1}^{2} & v_{1}^{2} & 1 & 0 & 0 & 0 & -u_{1}^{2} u_{2}^{2} & -v_{1}^{2} u_{2}^{2} &-u_{2}^{2} \\
0 & 0 & 0 & -u_{1}^{3} & -v_{1}^{3} & -1 & u_{1}^{3} v_{2}^{3} & v_{1}^{3} v_{2}^{3} &v_{2}^{3} \\
u_{1}^{3} & v_{1}^{3} & 1 & 0 & 0 & 0 & -u_{1}^{3} u_{2}^{3} & -v_{1}^{3} u_{2}^{3} &-u_{2}^{3} \\
0 & 0 & 0 & -u_{1}^{4} & -v_{1}^{4} & -1 & u_{1}^{4} v_{2}^{4} & v_{1}^{4} v_{2}^{4} &v_{2}^{4} \\
u_{1}^{4} & v_{1}^{4} & 1 & 0 & 0 & 0 & -u_{1}^{4} u_{2}^{4} & -v_{1}^{4} u_{2}^{4} &-u_{2}^{4} \\
\end{array}\right)}_{A} \underbrace{\left(\begin{array}{l}
h_{1} \\
h_{2} \\
h_{3} \\
h_{4} \\
h_{5} \\
h_{6} \\
h_{7} \\
h_{8} \\
h_{9}
\end{array}\right)}_{h}=0
$$
对$A$进行`SVD`分解($A=U\Sigma V$)，得到：
$$
h=V_{9}
$$
归一化前的单应性矩阵：
$$
x_{2}^{\prime}=Hx_{1}^{\prime} \Rightarrow x_{2}=\underbrace{T_{2}^{-1}HT_{1}}_{H^{\prime}} x_{1}
$$
```cpp
/**
 * @brief 从特征点匹配求homography（normalized DLT）
 * |x'|     | h1 h2 h3 ||x|
 * |y'| = a | h4 h5 h6 ||y|  简写: x' = a H x, a为一个尺度因子
 * |1 |     | h7 h8 h9 ||1|
 * 使用DLT(direct linear tranform)求解该模型
 * x' = a H x
 * ---> (x') 叉乘 (H x)  = 0
 * ---> Ah = 0
 * A = | 0  0  0 -x -y -1 xy' yy' y'|  h = | h1 h2 h3 h4 h5 h6 h7 h8 h9 |
 *     |-x -y -1  0  0  0 xx' yx' x'|
 * 通过SVD求解Ah = 0，A'A最小特征值对应的特征向量即为解
 * @param  vP1 归一化后的点, in reference frame
 * @param  vP2 归一化后的点, in current frame
 * @return     单应矩阵
 * @see        Multiple View Geometry in Computer Vision - Algorithm 4.2 p109
 */
Eigen::Matrix3f TwoViewReconstruction::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    Eigen::MatrixXf A(2 * N, 9);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A(2 * i, 0) = 0.0;
        A(2 * i, 1) = 0.0;
        A(2 * i, 2) = 0.0;
        A(2 * i, 3) = -u1;
        A(2 * i, 4) = -v1;
        A(2 * i, 5) = -1;
        A(2 * i, 6) = v2 * u1;
        A(2 * i, 7) = v2 * v1;
        A(2 * i, 8) = v2;

        A(2 * i + 1, 0) = u1;
        A(2 * i + 1, 1) = v1;
        A(2 * i + 1, 2) = 1;
        A(2 * i + 1, 3) = 0.0;
        A(2 * i + 1, 4) = 0.0;
        A(2 * i + 1, 5) = 0.0;
        A(2 * i + 1, 6) = -u2 * u1;
        A(2 * i + 1, 7) = -u2 * v1;
        A(2 * i + 1, 8) = -u2;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> H(svd.matrixV().col(8).data());

    return H;
}
```
* 双向校验：

由$\tilde{x}_{1}=H_{12}\cdot x_{2}$可以得到距离为：
$$
e = \left(\left(u_{1}-\tilde{u}_{1}\right)^2+ \left(v_{1}-\tilde{v}_{1}\right)^2\right)\cdot \frac{1}{\sigma^{2}}<th
$$
由$\tilde{x}_{2}=x_{1} \cdot H_{12}$可以得到距离为：
$$
e = \left(\left(u_{1}-\tilde{u}_{1}\right)^2+ \left(v_{1}-\tilde{v}_{1}\right)^2\right)\cdot \frac{1}{\sigma^{2}}<th
$$
```cpp
/** 
 * @brief 检查结果
 * @param H21 顾名思义
 * @param H12 顾名思义
 * @param vbMatchesInliers 匹配是否合法，大小为mvMatches12
 * @param sigma 默认为1
 */
float TwoViewReconstruction::CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float h11 = H21(0, 0);
    const float h12 = H21(0, 1);
    const float h13 = H21(0, 2);
    const float h21 = H21(1, 0);
    const float h22 = H21(1, 1);
    const float h23 = H21(1, 2);
    const float h31 = H21(2, 0);
    const float h32 = H21(2, 1);
    const float h33 = H21(2, 2);

    const float h11inv = H12(0, 0);
    const float h12inv = H12(0, 1);
    const float h13inv = H12(0, 2);
    const float h21inv = H12(1, 0);
    const float h22inv = H12(1, 1);
    const float h23inv = H12(1, 2);
    const float h31inv = H12(2, 0);
    const float h32inv = H12(2, 1);
    const float h33inv = H12(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;
    // 基于卡方检验计算出的阈值 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float th = 5.991;

    const float invSigmaSquare = 1.0 / (sigma * sigma);

    for (int i = 0; i < N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        // 计算投影误差，2投1 1投2这么做，计算累计的卡方检验分数，分数越高证明内点与误差越优，这么做为了平衡误差与内点个数，不是说内点个数越高越好，也不是说误差越小越好
        const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

        const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
        const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
        const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

        const float squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += th - chiSquare2;

        if (bIn)
            vbMatchesInliers[i] = true;
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}
```
#### 5.分数比率
$$
{R}_{H}=\frac{S_H}{S_H+S_F}
$$
* ${R}_{H} > 0.5$ ，选择H矩阵进行运动恢复
* ${R}_{H} \le 0.5$，选择F矩阵进行运动恢复
#### 6.重构
通过**H**或者**F**恢复两帧之间的运动，具体公式推导，后面文章单独给出
```cpp
    if (RH > 0.50) // if(RH>0.40)
    {
        // cout << "Initialization from Homography" << endl;
        return ReconstructH(vbMatchesInliersH, H, mK, T21, vP3D, vbTriangulated, minParallax, 50);
    }
    else // if(pF_HF>0.6)
    {
        // cout << "Initialization from Fundamental" << endl;
        return ReconstructF(vbMatchesInliersF, F, mK, T21, vP3D, vbTriangulated, minParallax, 50);
    }
```
### 5.CreateInitialMapMonocular
![在这里插入图片描述](https://img-blog.csdnimg.cn/a05699d004e14a62a7303430195414e1.png)

`CreateInitialMapMonocular`利用单目初始化的**F1帧**与**F2帧**生成关键帧，并生成**地图点MapPoints**，来构建**初始的地图**，其步骤为：
1. 将单目初始化的**当前帧**和**初始帧**创建为关键帧
2. 计算**当前关键帧**和**初始关键帧**的**Bow**
3. 将**当前关键帧**和**初始关键帧**添加到**地图**中
4. 生成地图点MapPoints
	1. 对于关键帧：
		* 添加地图点到关键帧中(AddMapPoint)：表示当前关键帧的第**i**个特征点对应的地图点为pMP
	2. 对于地图点：
		* 添加观测[AddObservation](ORB_SLAM3_地图点.md#2.观测信息)：该MapPoint可以被pKF关键帧的第i个特征点观测到
		* 计算代表性描述子[ComputeDistinctiveDescriptors](ORB_SLAM3_地图点.md#3.更新法向量和深度)
		* 更新法向量和深度[UpdateNormalAndDepth](ORB_SLAM3_地图点.md#4.计算地图点的最佳描述子)
	3. 添加**地图点**到**地图**中
5. 对于关键帧，更新共视关系
6. 全局BA优化[GlobalBundleAdjustemnt](ORB_SLAM3_优化方法.md#GlobalBundleAdjustemnt)
7. 用**当前关键帧所有地图点的深度的中值**进行**尺度归一化**
	1. 将当前关键帧的Pose进行归一化
	2. 将地图点的位置进行归一化，并更新法向量和深度[UpdateNormalAndDepth](ORB_SLAM3_地图点.md#4.计算地图点的最佳描述子)
8. 添加IMU预积分`mpImuPreintegratedFromLastKF`
9. 将关键帧插入到局部地图`mpLocalMapper`中
```cpp
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames 认为单目初始化时候的参考帧和当前帧都是关键帧
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);

    // Step 1 将初始关键帧,当前关键帧的描述子转为BoW
    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    // Step 2 将关键帧插入到地图
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    // Step 3 用初始化得到的3D点来生成地图点MapPoints
    //  mvIniMatches[i] 表示初始化两帧特征点匹配关系。
    //  具体解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值,没有匹配关系的话，vMatches12[i]值为 -1
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        // 没有匹配，跳过
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        // 用三角化点初始化为空间点的世界坐标
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        // Step 3.1 用3D点构造地图点
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        // Step 3.2 为该MapPoint添加属性：
        // a.观测到该MapPoint的关键帧
        // b.该MapPoint的描述子
        // c.该MapPoint的平均观测方向和深度范围

        // 表示该KeyFrame的2D特征点和对应的3D地图点
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        // b.从众多观测到该MapPoint的特征点中挑选最有代表性的描述子
        pMP->ComputeDistinctiveDescriptors();

        // c.更新该MapPoint平均观测方向以及观测距离的范围
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        // mvIniMatches下标i表示在初始化参考帧中的特征点的序号
        // mvIniMatches[i]是初始化当前帧中的特征点的序号
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    // Step 3.3 更新关键帧间的连接关系
    // 在3D点和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧公共3D点的个数
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    // Step 4 全局BA优化，同时优化所有位姿和三维点
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    // Step 5 取场景的中值深度，用于尺度归一化 
    // 为什么是 pKFini 而不是 pKCur ? 答：都可以的，内部做了位姿变换了
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    // 两个条件,一个是平均深度要大于0,另外一个是在当前帧中被观测到的地图点的数目应该大于50
    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
        mpSystem->ResetActiveMap();
        return;
    }

    // Step 6 将两帧之间的变换归一化到平均深度1的尺度下
    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    // x/z y/z 将z归一化到1 
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    // Step 7 把3D点的尺度也归一化到1
    // 为什么是pKFini? 是不是就算是使用 pKFcur 得到的结果也是相同的? 答：是的，因为是同样的三维点
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }

    // Step 8 将关键帧插入局部地图，更新归一化后的位姿、局部地图点
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    //mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    // 单目初始化之后，得到的初始地图中的所有点都是局部地图点
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    // 初始化成功，至此，初始化过程完成
    mState=OK;

    initID = pKFcur->mnId;
}
```


