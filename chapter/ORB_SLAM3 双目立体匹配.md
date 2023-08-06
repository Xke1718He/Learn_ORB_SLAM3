这里介绍ORB_SLAM算法中的双目匹配算法
# 双目匹配
在ORB_SLAM采用**粗匹配**和**精匹配**结合的方式来实现双目特征点的精确匹配
### 1.粗匹配
假设双目相机已被标定好，即双目相机的左右图像的**极线相互平行**，那么理论上左右图像中共轭点的坐标仅在u轴上不同
![ ](https://img-blog.csdnimg.cn/b1782af4edca4aabb94136a5536ab0b4.png)
为了寻找左图中的某一特征点在右图中对应的匹配点， 可以通过沿该特征点对应的极线进行搜索，这样避免了在整幅右图像上进行搜索，同时提高了匹配的速度。

然而， 由于误差的影响， 匹配点对的坐标在v方向可能存在几个像素的偏差，这样会造成沿极线搜索时无法匹配到最佳的特征点。因此，为了减少误差的影响，沿极线在v方向扩展一定的范围形成一个**带状区域**，将右图中待匹配的特征点按y 坐标分配到相应的带状区域
![在这里插入图片描述](https://img-blog.csdnimg.cn/ffbf14c7811b4333b796505b7a917494.png)
```cpp
    // 金字塔顶层（0层）图像高 nRows
    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    // Assign keypoints to row table
    // vRowIndices[0] = [1，2，5，8, 11]
    // vRowIndices[1] = [2，6，7，9, 13, 17, 20]
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    // 右图特征点数量，N表示数量 r表示右图，且不能被修改
    const int Nr = mvKeysRight.size();

    // Step 1. 行特征点统计. 考虑到尺度金字塔特征，一个特征点可能存在于多行，而非唯一的一行
    for(int iR=0; iR<Nr; iR++)
    {
        // 获取特征点ir的y坐标，即行号
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算特征点ir在行方向上，可能的偏移范围r，即可能的行号为[kpY + r, kpY -r]
        // 2 表示在全尺寸(scale = 1)的情况下，假设有2个像素的偏移，随着尺度变化，r也跟着变化
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        // 将特征点ir保证在可能的行号中
        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }
```

选取左图像中的某一特征点，如`a`，与对应的带状区域内中的点，如`b, c, d, e, f`，逐一进行描述子的比较，寻找最佳的匹配点。其中，待匹配集合中的特征点的x坐标理论上应该落在$\left[minU_{r}, maxU_{r}\right]$范围内，而$\left[minU_{r}, maxU_{r}\right]$可以通过公式确 定。 因 此，可以预先 筛选掉落在$\left[minU_{r}, maxU_{r}\right]$之外的点`b`。
$$
\left\{\begin{array} { l } 
{ \operatorname { max } d = \frac { f \cdot B } { \operatorname { m i n } Z } } \\
{ \operatorname { min } d = \frac { f \cdot B } { \operatorname { m a x } Z } }
\end{array} \Rightarrow \left\{\begin{array}{l}
\max U_{r}=U_{l}-\min d \\
\min U_{r}=U_{l}-\max d
\end{array}\right.\right.
$$

```cpp
    // 为左图每一个特征点il，在右图搜索最相似的特征点ir
    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        // 获取左图特征点il所在行，以及在右图对应行中可能的匹配点
        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        // 计算理论上的最佳搜索范围
        const float minU = uL-maxD;
        const float maxU = uL-minD;

        // 最大搜索范围小于0，说明无匹配点
        if(maxU<0)
            continue;

        // 初始化最佳相似度，用最大相似度，以及最佳匹配点索引
        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        // Step2. 粗配准. 左图特征点il与右图中的可能的匹配点进行逐个比较,得到最相似匹配点的相似度和索引
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            // 左图特征点il与带匹配点ic的空间尺度差超过2，放弃
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            // 使用列坐标(x)进行匹配，和stereomatch一样
            const float &uR = kpR.pt.x;

            // 超出理论搜索范围[minU, maxU]，可能是误匹配，放弃
            if(uR>=minU && uR<=maxU)
            {
                // 计算匹配点il和待匹配点ic的相似度dist
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                // 统计最小相似度及其对应的列坐标(x)
                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }
    .......
   }
```

### 2.精匹配
在经过了**粗匹配**后， 左图像中的特征点在右图像中对应的带状区域内寻找到了**最佳的匹配点**，但是由于特征点的坐标可能不是那么精准，**此时的匹配点可能并不是真正的最佳匹配点**， 因此需要在粗匹配的基础上进一步进行精匹配。 通过在当前匹配点的位置附近构建**滑动窗口**，然后使用**模板匹配**算法寻找更优的位置
![在这里插入图片描述](https://img-blog.csdnimg.cn/572ced0558df4796880791613cfe2478.png)
**左右红点**是双目图像通过粗匹配后得到的匹配点对，以左图的匹配点为中心取一个宽为W的图像块，然后在右图中构建一个相同尺寸的图像块， 在搜索范围$\left[-L, L\right]$内从左向右滑动窗口， 并通过模板匹配算法依次计算两个图像块的相似度， 最终找到最佳的匹配位置， 其中相似度计算:
$$
S A D(u, v, l)=\sum_{i=-\frac{w}{2}}^{\frac{w}{2}} \sum_{j=-\frac{w}{2}}^{\frac{w}{2}}\left|I_{l}(u+i, v+j)-I_{r}(u+i+l, v+j)\right|, l \in(-L, L)
$$
得到最佳匹配后，进行**亚像素**插值，用最佳匹配点及其左右两边的相邻点进行抛物线拟合
$$
\begin{array}{c}
\Delta R = \frac{dist_{L}-dist_{R}}{2\times \left(dist_{L}+dist_{R}-2\times bestDist\right)} \\
u_{R}^{'} = u_{R} + R + \Delta R \\
R\in \left [-L, L\right ] 
\end{array}
$$
其中，$R$是滑动窗口**最佳匹配位置**移动的**偏移量**
```cpp
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // 计算右图特征点x坐标和对应的金字塔尺度
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            // 尺度缩放后的左右图特征点坐标
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            // 滑动窗口搜索, 类似模版卷积或滤波
            // w表示sad相似度的窗口半径
            const int w = 5;
            // 提取左图中，以特征点(scaleduL,scaledvL)为中心, 半径为w的图像快patch
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);

            // 初始化最佳相似度
            int bestDist = INT_MAX;
			// 通过滑动窗口搜索优化，得到的列坐标偏移量
            int bestincR = 0;
			// 滑动窗口的滑动范围为（-L, L）
            const int L = 5;
			// 初始化存储图像块相似度
            vector<float> vDists;
            vDists.resize(2*L+1);

            // 计算滑动窗口滑动范围的边界，因为是块匹配，还要算上图像块的尺寸
            // 列方向起点 iniu = r0 + 最大窗口滑动范围 - 图像块尺寸
            // 列方向终点 eniu = r0 + 最大窗口滑动范围 + 图像块尺寸 + 1
            // 此次 + 1 和下面的提取图像块是列坐标+1是一样的，保证提取的图像块的宽是2 * w + 1
            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            // 判断搜索是否越界
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            // 在搜索范围内从左到右滑动，并计算图像块相似度
            for(int incR=-L; incR<=+L; incR++)
            {
                // 提取左图中，以特征点(scaleduL,scaledvL)为中心, 半径为w的图像快patch
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);

                // sad 计算
                float dist = cv::norm(IL,IR,cv::NORM_L1);
                // 统计最小sad和偏移量
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                // L+incR 为refine后的匹配点列坐标(x)
                vDists[L+incR] = dist;
            }

            // 搜索窗口越界判断ß 
            if(bestincR==-L || bestincR==L)
                continue;

            // Step 4. 亚像素插值, 使用最佳匹配点及其左右相邻点构成抛物线
            // 使用3点拟合抛物线的方式，用极小值代替之前计算的最优是差值
            //    \                 / <- 由视差为14，15，16的相似度拟合的抛物线
            //      .             .(16)
            //         .14     .(15) <- int/uchar最佳视差值
            //              . 
            //           （14.5）<- 真实的视差值
            //   deltaR = 15.5 - 16 = -0.5
            // 公式参考opencv sgbm源码中的亚像素插值公式
            // 或论文<<On Building an Accurate Stereo Matching System on Graphics Hardware>> 公式7
            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            // 亚像素精度的修正量应该是在[-1,1]之间，否则就是误匹配
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 根据亚像素精度偏移量delta调整最佳匹配索引
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                // 如果存在负视差，则约束为0.01
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                // 根据视差值计算深度信息
                // 保存最相似点的列坐标(x)信息
                // 保存归一化sad最小相似度
                // Step 5. 最优视差值/深度选择.
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
```
### 3.完整代码
```cpp
void Frame::ComputeStereoMatches()
{
    /*两帧图像稀疏立体匹配
     * 输入：两帧立体矫正后的图像img_left 和 img_right 对应的orb特征点集
     * 过程：
          1. 行特征点统计. 统计img_right每一行上的ORB特征点集，便于使用立体匹配思路(行搜索/极线搜索）进行同名点搜索, 避免逐像素的判断.
          2. 粗匹配. 根据步骤1的结果，对img_left第i行的orb特征点pi，在img_right的第i行上的orb特征点集中搜索相似orb特征点, 得到qi
          3. 精确匹配. 以点qi为中心，半径为r的范围内，进行块匹配（归一化SAD），进一步优化匹配结果
          4. 亚像素精度优化. 步骤3得到的视差为uchar/int类型精度，并不一定是真实视差，通过亚像素差值（抛物线插值)获取float精度的真实视差
          5. 最优视差值/深度选择. 通过胜者为王算法（WTA）获取最佳匹配点。
          6. 删除离缺点(outliers). 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是正确匹配，比如光照变化、弱纹理等会造成误匹配
     * 输出：稀疏特征点视差图/深度图（亚像素精度）mvDepth 匹配结果 mvuRight
     */

    // 为匹配结果预先分配内存，数据类型为float型
    // mvuRight存储右图匹配点索引
    // mvDepth存储特征点的深度信息
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    // orb特征相似度阈值  -> mean ～= (max  + min) / 2
    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    // 金字塔顶层（0层）图像高 nRows
    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    // Assign keypoints to row table
    // 二维vector存储每一行的orb特征点的列坐标的索引，为什么是vector，因为每一行的特征点有可能不一样，例如
    // vRowIndices[0] = [1，2，5，8, 11]   第1行有5个特征点,他们的列号（即x坐标）分别是1,2,5,8,11
    // vRowIndices[1] = [2，6，7，9, 13, 17, 20]  第2行有7个特征点.etc
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    // 右图特征点数量，N表示数量 r表示右图，且不能被修改
    const int Nr = mvKeysRight.size();

    // Step 1. 行特征点统计. 考虑到尺度金字塔特征，一个特征点可能存在于多行，而非唯一的一行
    for(int iR=0; iR<Nr; iR++)
    {
        // 获取特征点ir的y坐标，即行号
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算特征点ir在行方向上，可能的偏移范围r，即可能的行号为[kpY + r, kpY -r]
        // 2 表示在全尺寸(scale = 1)的情况下，假设有2个像素的偏移，随着尺度变化，r也跟着变化
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        // 将特征点ir保证在可能的行号中
        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Step 2 -> 3. 粗匹配 + 精匹配
    // 对于立体矫正后的两张图，在列方向(x)存在最大视差maxd和最小视差mind
    // 也即是左图中任何一点p，在右图上的匹配点的范围为应该是[p - maxd, p - mind], 而不需要遍历每一行所有的像素
    // maxd = baseline * length_focal / minZ
    // mind = baseline * length_focal / maxZ
    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    // 保存sad块匹配相似度和左图特征点索引
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    // 为左图每一个特征点il，在右图搜索最相似的特征点ir
    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        // 获取左图特征点il所在行，以及在右图对应行中可能的匹配点
        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        // 计算理论上的最佳搜索范围
        const float minU = uL-maxD;
        const float maxU = uL-minD;

        // 最大搜索范围小于0，说明无匹配点
        if(maxU<0)
            continue;

        // 初始化最佳相似度，用最大相似度，以及最佳匹配点索引
        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        // Step2. 粗配准. 左图特征点il与右图中的可能的匹配点进行逐个比较,得到最相似匹配点的相似度和索引
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            // 左图特征点il与带匹配点ic的空间尺度差超过2，放弃
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            // 使用列坐标(x)进行匹配，和stereomatch一样
            const float &uR = kpR.pt.x;

            // 超出理论搜索范围[minU, maxU]，可能是误匹配，放弃
            if(uR>=minU && uR<=maxU)
            {
                // 计算匹配点il和待匹配点ic的相似度dist
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                // 统计最小相似度及其对应的列坐标(x)
                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        // 如果刚才匹配过程中的最佳描述子距离小于给定的阈值
        // Step 3. 精确匹配. 
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // 计算右图特征点x坐标和对应的金字塔尺度
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            // 尺度缩放后的左右图特征点坐标
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            // 滑动窗口搜索, 类似模版卷积或滤波
            // w表示sad相似度的窗口半径
            const int w = 5;
            // 提取左图中，以特征点(scaleduL,scaledvL)为中心, 半径为w的图像快patch
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);

            // 初始化最佳相似度
            int bestDist = INT_MAX;
			// 通过滑动窗口搜索优化，得到的列坐标偏移量
            int bestincR = 0;
			// 滑动窗口的滑动范围为（-L, L）
            const int L = 5;
			// 初始化存储图像块相似度
            vector<float> vDists;
            vDists.resize(2*L+1);

            // 计算滑动窗口滑动范围的边界，因为是块匹配，还要算上图像块的尺寸
            // 列方向起点 iniu = r0 + 最大窗口滑动范围 - 图像块尺寸
            // 列方向终点 eniu = r0 + 最大窗口滑动范围 + 图像块尺寸 + 1
            // 此次 + 1 和下面的提取图像块是列坐标+1是一样的，保证提取的图像块的宽是2 * w + 1
            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            // 判断搜索是否越界
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            // 在搜索范围内从左到右滑动，并计算图像块相似度
            for(int incR=-L; incR<=+L; incR++)
            {
                // 提取左图中，以特征点(scaleduL,scaledvL)为中心, 半径为w的图像快patch
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);

                // sad 计算
                float dist = cv::norm(IL,IR,cv::NORM_L1);
                // 统计最小sad和偏移量
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                // L+incR 为refine后的匹配点列坐标(x)
                vDists[L+incR] = dist;
            }

            // 搜索窗口越界判断ß 
            if(bestincR==-L || bestincR==L)
                continue;

            // Step 4. 亚像素插值, 使用最佳匹配点及其左右相邻点构成抛物线
            // 使用3点拟合抛物线的方式，用极小值代替之前计算的最优是差值
            //    \                 / <- 由视差为14，15，16的相似度拟合的抛物线
            //      .             .(16)
            //         .14     .(15) <- int/uchar最佳视差值
            //              . 
            //           （14.5）<- 真实的视差值
            //   deltaR = 15.5 - 16 = -0.5
            // 公式参考opencv sgbm源码中的亚像素插值公式
            // 或论文<<On Building an Accurate Stereo Matching System on Graphics Hardware>> 公式7
            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            // 亚像素精度的修正量应该是在[-1,1]之间，否则就是误匹配
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 根据亚像素精度偏移量delta调整最佳匹配索引
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                // 如果存在负视差，则约束为0.01
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                // 根据视差值计算深度信息
                // 保存最相似点的列坐标(x)信息
                // 保存归一化sad最小相似度
                // Step 5. 最优视差值/深度选择.
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    // Step 6. 删除离缺点(outliers)
    // 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是匹配的，比如光照变化、弱纹理、无纹理等同样会造成误匹配
    // 误匹配判断条件  norm_sad > 1.5 * 1.4 * median
    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            // 误匹配点置为-1，和初始化时保持一直，作为error code
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}
```


