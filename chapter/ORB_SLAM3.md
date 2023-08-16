# ORB_SLAM3 系统初始化

ORB_SLAM3的系统初始化主要是：

* 从配置文件中读取需要的**参数**
* 创建**跟踪线程，局部建图线程，闭环线程**三大主线程，以及可视化线程
* 创建后面需要的对象：**ORB词袋**(`ORBVocabulary`)、**关键帧数据库**(`KeyFrameDatabase`)、**多地图**(`Atlas`)等

其步骤如下：
1. 检测配置文件能否打开
2. 加载**ORB词袋**(`ORBVocabulary`)
3. 创建**关键帧数据库**(`KeyFrameDatabase`)
4. 创建**多地图**(`Atlas`)
5. 创建**跟踪线程**(`Tracking`)
6. 创建**局部建图线程**(`LocalMapping`)
7. 创建**闭环线程**(`LoopClosing`)
8. 设置**线程间的指针**

这里主要讲解**跟踪线程**的初始化，而对于**局部建图线程，闭环线程**的初始化只是简单的参数赋值
## 跟踪线程
```cpp
mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                         mpAtlas, mpKeyFrameDatabase, strSettingsFile, 
                         mSensor, settings_, strSequence);
```
### A.相机模型
* 从配置文件中读取`相机参数`
```cpp
bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
```
* 创建相机模型(`Pinhole/KannalaBrandt8`)
    * `Pinhole/KannalaBrandt8`： 继承自`GeometricCamera`，输入：相机参数（`fx,fy,cx,cy, k1, k2, p1, p2, k3`/`fx,fy,cx,cy,k1,k2,k3,k4`）
    * 添加相机模型到多地图系统中(`mpAtlas->AddCamera(mpCamera)`)
	* 对于针孔相机模型`Pinhole`可以参考文章：[【二】[详细]针孔相机模型、相机镜头畸变模型、相机标定与OpenCV实现](https://blog.csdn.net/He3he3he/article/details/98769173)
* `GeometricCamera`
```cpp
    class GeometricCamera {
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnType;
        ar & mvParameters;
    }
public:
    //构造函数
    GeometricCamera() {}
    GeometricCamera(const std::vector<float> &_vParameters) : mvParameters(_vParameters) {}
    //析构函数
    ~GeometricCamera() {}
    //3D->2D
    virtual cv::Point2f project(const cv::Point3f &p3D) = 0;
    virtual Eigen::Vector2d project(const Eigen::Vector3d & v3D) = 0;
    virtual Eigen::Vector2f project(const Eigen::Vector3f & v3D) = 0;
    virtual Eigen::Vector2f projectMat(const cv::Point3f& p3D) = 0;

    virtual float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) = 0;
     //2d->3d
    virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) = 0;
    virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;

    virtual Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D) = 0;

    virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12, Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated) = 0;

    virtual cv::Mat toK() = 0;
    virtual Eigen::Matrix3f toK_() = 0;

    virtual bool epipolarConstrain(GeometricCamera* otherCamera, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc) = 0;

    float getParameter(const int i){return mvParameters[i];}
    void setParameter(const float p, const size_t i){mvParameters[i] = p;}

    size_t size(){return mvParameters.size();}

    virtual bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther, Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2, const float sigmaLevel1, const float sigmaLevel2, Eigen::Vector3f& x3Dtriangulated) = 0;

    unsigned int GetId() { return mnId; }

    unsigned int GetType() { return mnType; }

    const static unsigned int CAM_PINHOLE = 0;
    const static unsigned int CAM_FISHEYE = 1;

    static long unsigned int nNextId;

protected:
    std::vector<float> mvParameters;

    unsigned int mnId;

    unsigned int mnType;
    };
```

### B.ORB特征提取器

从配置文件中读取`ORB参数`，其主要参数

* **nFeatures**: 特征点的数量
* **Scalefactor**：特征金字塔的尺度因子
* **nLevels**: 特征金字塔的层数
* **iniThFAST**: 初始Fast阈值
* **minThFAST**: 最小Fast阈值

```cpp
bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
```
创建ORB特征提取器(`ORBextractor`)，`ORBextractor`的构造函数如下

```cpp
ORBextractor(int nfeatures, float scaleFactor, int nlevels,
             int iniThFAST, int minThFAST);
```
需要注意的是单目情况下，特征点的数目增加为**5**倍
```cpp
mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
//相对于普通情况，单目初始化时提取的特征点数量更多
mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
```
构造`ORBextractor`特征提取器主要分为如下几个步骤：
1. 设置图像金字塔的`尺度因子`、`逆尺度因子`、`方差`
$$
\begin{array}{l}
mvScalefactor_{i} = mvScalefactor_{i-1} \cdot scalefactor \\
mvInvScalefactor_{i} = \frac{1}{mvScalefactor_{i}} \\
mvLevelSigma2_{i} = mvScalefactor_{i}^{2} \\
mvInvLevelSigma2_{i} = \frac{1}{mvLevelSigma2_{i}} 
\end{array}
$$
```C++
    mvScaleFactor.resize(nlevels);  
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)  
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }
```
2. 构造图像金字塔，**图像金字塔**如下：
    ![](https://img-blog.csdnimg.cn/img_convert/740b8d6c077f469ac02e1c9f0ca4265a.png)
3. `预分配`每层金字塔的特征点数量，其分配方式：
   * 先分配前$n-1$层，再将剩余的特征点$N-sum(0,n-1)$分配给第n层。


每层金字塔期望的特征点数量：

假设需要提取的特征点数量为$N$，金字塔共有$m$层，第0层图像的宽为$W$，高为$H$ ，对应的面积$H\cdot W =C$，图像金字塔缩放因子为$s(0<s<1)$，在 ORB-SLAM中 ，$m=8，s=1/ 1.2$。
那么，整个金字塔总面积为：
$$
\begin{aligned}
S &=H \cdot W \cdot\left(s^{2}\right)^{0}+H \cdot W \cdot\left(s^{2}\right)^{1}+\cdots+H \cdot W \cdot\left(s^{2}\right)^{(m-1)} \\
&=H W \frac{1-\left(s^{2}\right)^{m}}{1-s^{2}}=C \frac{1-\left(s^{2}\right)^{m}}{1-s^{2}}
\end{aligned}
$$
单位面积应该分配的特征点数目为：
$$
N_{a v g}=\frac{N}{S}=\frac{N}{C \frac{1-\left(s^{2}\right)^{m}}{1-s^{2}}}=\frac{N\left(1-s^{2}\right)}{C\left(1-\left(s^{2}\right)^{m}\right)}
$$
第0层应该分配的特征点数量为：
$$
N_{0}=\frac{N\left(1-s^{2}\right)}{1-\left(s^{2}\right)^{m}}
$$
第 i 层应该分配的特征点数量为：
$$
N_{i}=\frac{N\left(1-s^{2}\right)}{C\left(1-\left(s^{2}\right)^{m}\right)} C\left(s^{2}\right)^{i}=\frac{N\left(1-s^{2}\right)}{1-\left(s^{2}\right)^{m}}\left(s^{2}\right)^{i}
$$
在ORB-SLAM3 的代码里，**不是按照面积均摊的，而是按照面积的开方**来均摊特征点的，也就是将上述公式中的$s^2$换成$s$ 即可。
```cpp
mnFeaturesPerLevel.resize(nlevels);
float factor = 1.0f / scaleFactor;
float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

int sumFeatures = 0;
for( int level = 0; level < nlevels-1; level++ )
{
    mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
    sumFeatures += mnFeaturesPerLevel[level];
    nDesiredFeaturesPerScale *= factor;
}
mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);
```

4. 初始化pattern
pattern为$32\cdot8\cdot4 = 1024$，其中pattern是ORB预设好的。
参考文献：《BRIEF: Binary Robust Independent Elementary Features 》
![](https://img-blog.csdnimg.cn/img_convert/b62ce577715e34aca0ee591b3c67fd33.png)

```cpp
    const int npoints = 512;
    const Point* pattern0 = (const Point*)bit_pattern_31_;	
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));
```
5. 预先计算灰度质心法每行对应的终点
	1. 先算弧`A->B`($v\in(0,vmax )$)段
	2. 再算弧`C->B`($v\in(r,vmin )$)段

```cpp
	//umax存放u轴的最大值，因为只有1/4圆，所以size为半径+1
   umax.resize(HALF_PATCH_SIZE + 1);
	
    int v,v0,vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);//为v轴根号2/2处,也就是
    
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
	//半径的平方
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;

	//计算每行的umax
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
	//利用对称性，计算第一象限的另外的1/8圆的umax
	for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
```
![](https://img-blog.csdnimg.cn/img_convert/e0ed72ff1ad10da140f9bd1f3a4595a9.png)
注意：这里算出来`vmax`和`vmin`是相等的，可以看打印出来的点中有一个红圈套篮圈
![](https://img-blog.csdnimg.cn/img_convert/965d6697172ff844656654e069a4d074.png)

### C.IMU预积分
从配置文件中读取IMU参数，创建IMU标定(`IMU::Calib`)、IMU预积分(`IMU::Preintegrated`)
* Tbc：相机到IMU的变换矩阵
* freq：IMU的频率
	* 单位：$hz$
	* 符号：$\frac{1}{\Delta t}$
* Ng：陀螺仪白噪声
	* 单位：$\frac{\text { rad }}{s} \frac{1}{\sqrt{H_{z}}}$
	* 符号：$\sigma_{g}$
* Na：加速计白噪声
	* 单位：$\frac{m}{s^{2}} \frac{1}{\sqrt{H z}}$
	* 符号：$\sigma_{a}$
* Ngw：陀螺仪随机游走噪声
	* 单位：$\frac{\text { rad }}{s^{2}} \frac{1}{\sqrt{H z}}$
	* 符号：$\sigma_{bg}$
* Naw：加速计随机游走噪声
	* 单位：$\frac{m}{s^{3}} \frac{1}{\sqrt{H z}}$
	* 符号：$\sigma_{ba}$

这里主要创建**来自上一关键帧的IMU预积分**，主要参数为**高斯白噪声协方差**和**随机游走协方差**：
```cpp
mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);
mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
```
高斯白噪声的方差从连续时间到离散时间需要乘以一个$\frac{1}{\sqrt{freq}}$   
$$
\begin{array}{c}
N_{a} = N_{a}^{d}\sqrt{freq} \\
N_{g} = N_{g}^{d}\sqrt{freq}
\end{array}
$$
随机游走方差从连续时间到离散时间乘以一个 $\sqrt{freq}$
$$
\begin{array}{c}
N_{aw} = N_{aw}^{d}\frac{1}{\sqrt{freq}} \\
N_{gw} = N_{gw}^{d}\frac{1}{\sqrt{freq}}
\end{array}
$$
高斯白噪声协方差Cov：
$$
\begin{bmatrix}
N_{g}^{2} & 0 & 0 & 0 & 0 & 0\\
0  & N_{g}^{2} & 0 & 0 & 0 & 0\\
0  & 0 &N_{g}^{2}  & 0 & 0 & 0\\
 0 &  0& 0 &N_{a}^{2}  &  0& 0\\
  0&  0&  0&  0&N_{a}^{2}  & 0\\
  0&  0&  0&  0&  0&N_{a}^{2}
\end{bmatrix}
$$
随机游走协方差CovWalk：
$$
\begin{bmatrix}
N_{gw}^{2} & 0 & 0 & 0 & 0 & 0\\
0  & N_{gw}^{2} & 0 & 0 & 0 & 0\\
0  & 0 &N_{gw}^{2}  & 0 & 0 & 0\\
 0 &  0& 0 &N_{aw}^{2}  &  0& 0\\
  0&  0&  0&  0&N_{aw}^{2}  & 0\\
  0&  0&  0&  0&  0&N_{aw}^{2}
\end{bmatrix}
$$
```cpp
void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw)
{
    mbIsSet = true;
    const float ng2 = ng * ng;
    const float na2 = na * na;
    const float ngw2 = ngw * ngw;
    const float naw2 = naw * naw;

    mTbc = sophTbc;
    mTcb = mTbc.inverse();
    // 噪声协方差
    Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    // 随机游走协方差
    CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}
```
# 构造Frame

为了构建一帧**Frame**，主要的步骤如下：
1. 提取ORB特征点(`ExtractORB`)
2. 对提取的特征点进行矫正(`cv::undistortPoints`)
3. 计算去畸变后的图像边界(`ComputeImageBounds`)
4. 将特征点分配到网格中(`AssignFeaturesToGrid`)
## A.提取ORB特征点
首先需要对当前帧图像进行特征点的提取：

* 计算图像金字塔(`ComputePyramid`)
* 提取Fast角点(`ComputeKeyPointsOctTree`)
  * 四叉树均匀化(`DistributeOctTree`)
* 计算特征点的方向(`computeOrientation`)
* 计算特征点的描述子(`computeDescriptors`)

### 1.计算图像金字塔
```cpp
    void ORBextractor::ComputePyramid(cv::Mat image)
    {
        for (int level = 0; level < nlevels; ++level)
        {
            float scale = mvInvScaleFactor[level];
            Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
            Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
            Mat temp(wholeSize, image.type()), masktemp;
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));
            //0层以上
            if( level != 0 )
            {
                //将上一层金字塔图像根据设定sz缩放到当前层级
                resize(mvImagePyramid[level-1],	//输入图像
                       mvImagePyramid[level], 	//输出图像
                       sz, 					//输出图像的尺寸
                       0, 					//水平方向上的缩放系数，0表示自动计算
                       0,  					//垂直方向上的缩放系数，0表示自动计算
                       cv::INTER_LINEAR);	//线性插值


                copyMakeBorder(mvImagePyramid[level],
                               temp,
                               EDGE_THRESHOLD, EDGE_THRESHOLD,
                               EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101+BORDER_ISOLATED);
            }
            else
            {
                //对于0层图像，直接就扩充边界了
                copyMakeBorder(image,
                               temp,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               BORDER_REFLECT_101);            
            }
    	}
    }
```
* 对于level = 0
	* 对原图像的上下左右扩充`EDGE_THRESHOLD(19)`像素
* 对于level !=0
	* 对金字塔`level-1`层图像进行缩放`scale`得到`level`层图像
	* 对`level`层图像的边界上下左右扩充`EDGE_THRESHOLD(19)`像素
* 实际操作：只对原图像进行了一层一层的缩放
![在这里插入图片描述](https://img-blog.csdnimg.cn/8a5ffd166ffe449c8909f31f56445851.png)
```cpp
void cv::copyMakeBorder
(
	InputArray _src_, //输入图像
	OutputArray _dst_,//输出图像
	int _top_,//上边界的大小
	int _bottom_,//下边界的大小
	int _left_,//左边界的大小
	int _right_,//右边界的大小
	int _borderType_,//边界的类型
	const Scalar& _value_ = Scalar()//边界的值
	//BORDER_REPLICATE(复制):     aaaaaa|abcdefgh|hhhhhhh 
	//BORDER_REFLECT(镜像):       fedcba|abcdefgh|hgfedcb  
	//BORDER_REFLECT_101(镜像):   gfedcb|abcdefgh|gfedcba  
	//BORDER_WRAP(包裹):          cdefgh|abcdefgh|abcdefg  
	//BORDER_CONSTANT(常量):      iiiiii|abcdefgh|iiiiiii  with some specified i
)
```
### 2.提取Fast角点
对于金字塔的每一层，将其网格化，每个格子大小为$w = 35$：
* 左上角(**红色**)：$\left(minBorderX, minBorderY\right)$
* 右下角：$\left(maxBorderY, maxBorderY\right)$
* 网格的行数：$rows=\frac{\left(maxBorderY-minBorderY\right)}{w}$
* 网格的列数：$cols=\frac{\left(maxBorderX-minBorderX\right)}{w}$
* 网格的宽度：$wCell = ceil\left(\frac{maxBorderX-minBorderX}{cols}\right)$
* 网格的高度：$hCell = ceil\left(\frac{maxBorderY-minBorderY}{rows}\right)$

这时候可以在每个格子中提取Fast角点， 其中格子的范围为：
* $iniX = minBorderX + j \cdot wCell, j \in \left(0,nCols\right)$
* $iniY = minBorderY + i \cdot hCell, i\in \left(0,nRows\right)$
* $maxX = iniX + wCell + 6$
* $maxY = iniY + hCell + 6$

**FAST角点**在`(iniX, iniY, maxX, maxY)`范围内提取，这里使用**高低阈值**
```cpp
FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
	vKeysCell, iniThFAST, true);
if(vKeysCell.empty())
{
	FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),	
		 vKeysCell, minThFAST, true);
}
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/1094569515d9441fa15c6f7e2a9e8a66.png)
提取到的角点的原点位于`紫色`，需要将其变换到`红色`
$$
\begin{array}{c}
x = x + j \cdot wCell \\
y = y + i \cdot hCell
\end{array}
$$
```cpp
for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); 	vit!=vKeysCell.end();vit++)
{
    (*vit).pt.x+=j*wCell;
    (*vit).pt.y+=i*hCell;
    vToDistributeKeys.push_back(*vit);
}
```
### 3.四叉树均匀化
四叉树的节点如下：
```cpp
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys //该节点下的特征点;
    cv::Point2i UL, UR, BL, BR //上下左右边界;
    std::list<ExtractorNode>::iterator lit //迭代器，指向自己;
    bool bNoMore //当特征点数量为1是，不可再分裂;
};
```
步骤：
1. 根据`宽高比例`计算初始的节点数
2. 根据节点数生成初始化节点
3. 将特征点分配到对应的节点中(`kp.pt.x/hX`)
4. 对于初始化节点，标记那些不能分裂的，删除那些空的节点
5. 记录状态：
	* preSize: 当前节点特征点的数量
	* nToExpand: 需要分裂的节点数
	* vSizeAndPointerToNode: 可分裂的节点的指针及其特征点数
6. 分裂
	* 添加子节点
	* 将`可分裂`的子节点加入到`可分裂的节点`中
	* 特征点数量等于1的标记为不可分裂节点
	* 删除当前节点
7. 结束条件：
	* 当前的节点数已经超过了要求的特征点数
	* 当前所有的节点都不再分了
8. 对于: 当前节点数 + 即将分裂的节点数 * 3 > N
	* 对即将分裂的节点数按照特征点数量的多少进行`排序`
	* 按照`特征点数量多的节点先分裂`的原则进行分裂
	* 如果分裂过程中，满足`结束条件`，则立即结束。
9. 非极大值抑制

示例：目标特征点数(20)
1. 一个初始节点分为4个子节点
2. 4个节点分裂为16个子节点，其中：
	* 3个子节点中无特征点，删除；
	* 3个子节点中只有一个特征点，标记为不可分
	* 剩余10个子节点，添加到`可分裂的节点`中
3. 13个节点 + 10个可分节点 * 3 > 20
	* 对10个可分节点排序
	* 当节点数达到20，停止
4. 对每个节点进行非极大值抑制，选取响应最大的特征点作为该节点最终特征点

![在这里插入图片描述](https://img-blog.csdnimg.cn/ee53430bb6c34da29830a9f292cd68d8.png)
在四叉树均匀化后，将特征点坐标变换到`青色`
$$
\begin{array}{c}
x = x + minBorderX \\
y = y + minBorderY
\end{array}
$$
```cpp
        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];
        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=level;
            keypoints[i].size = scaledPatchSize;
        }
```
### 4.计算特征点方向
遍历所有金字塔层，计算当前金字塔层所有特征点的方向，步骤：
1. 计算图像在x, y方向的矩
$$
\begin{array}{l}
m_{10}=\sum_{x=-R}^{R} \sum_{y=-R}^{R} x I(x, y) \\
m_{01}=\sum_{x=-R}^{R} \sum_{y=-R}^{R} y I(x, y) \\
m_{00}=\sum_{x=-R}^{R} \sum_{y=-R}^{R} I(x, y)
\end{array}
$$
2. 图像的质心为：
$$
C=\left(c_{x}, c_{y}\right)=\left(\frac{m_{10}}{m_{00}}, \frac{m_{01}}{m_{00}}\right)
$$
3. 旋转角度：
$$
\theta=\arctan 2\left(c_{y}, c_{x}\right)=\arctan 2\left(m_{01}, m_{10}\right)
$$
![](https://img-blog.csdnimg.cn/13ed60b1068744dabd3ab92a29a76c9b.png)
```cpp
static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;

	//获得这个特征点所在的图像块的中心点坐标灰度值的指针center
    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circular patch  
    int step = (int)image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
		// 假设每次处理的两个点坐标，中心线下方为(x,y),中心线上方为(x,-y) 
        // 对于某次待处理的两个点：m_10 = Σ x*I(x,y) =  x*I(x,y) + x*I(x,-y) = x*(I(x,y) + I(x,-y))
        // 对于某次待处理的两个点：m_01 = Σ y*I(x,y) =  y*I(x,y) - y*I(x,-y) = y*(I(x,y) - I(x,-y))
        for (int u = -d; u <= d; ++u)
        {
			//得到需要进行加运算和减运算的像素灰度值
			//val_plus：在中心线下方x=u时的的像素灰度值
            //val_minus：在中心线上方x=u时的像素灰度值
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
			//在v（y轴）上，2行所有像素灰度值之差
            v_sum += (val_plus - val_minus);
			//u轴（也就是x轴）方向上用u坐标加权和（u坐标也有正负符号），相当于同时计算两行
            m_10 += u * (val_plus + val_minus);
        }
        //将这一行上的和按照y坐标加权
        m_01 += v * v_sum;
    }

    //为了加快速度还使用了fastAtan2()函数，输出为[0,360)角度，精度为0.3°
    return fastAtan2((float)m_01, (float)m_10);
}
```

**注意**: 矩的计算方式
矩在圆中以行的方式进行计算，首先计算v=0行，上下两行中对应的点为$(x, y)$与$(x, -y)$，则矩为：
$$
\begin{align}
m_{10} & = \sum xI(x,y) \\ & = xI(x,y) + xI(x,-y) \\ & = x(I(x,y) + I(x,-y)) \\
m_{01} & = \sum yI(x,y) \\ & = yI(x,y) - yI(x,-y) \\ & = y(I(x,y) - I(x,-y))
\end{align}
$$

### 5.计算特征点的描述子
遍历所有金字塔层，计算当前金字塔层所有特征点的描述子，步骤：
1. 高斯模糊
2. 旋转
$$
\boldsymbol{Q}_{\theta}=\boldsymbol{R}_{\theta} \boldsymbol{Q}
$$
3. 计算描述子
$$
\boldsymbol{Q_{\theta}}=\left(\begin{array}{c}
x_{1}, x_{2}, \cdots, x_{m-1}, x_{m} \\
y_{1}, y_{2}, \cdots, y_{m-1} y_{m}
\end{array}\right)
$$

选取`32`组点对，每组点对包含`16`个点，两两相互比较，生成`8`位，所以描述子维度：$32\cdot8=256$
```cpp
static void computeOrbDescriptor(const KeyPoint& kpt,
                                 const Mat& img, const Point* pattern,
                                 uchar* desc)
{
	//得到特征点的角度，用弧度制表示。kpt.angle是角度制，范围为[0,360)度
    float angle = (float)kpt.angle*factorPI;
	//然后计算这个角度的余弦值和正弦值
    float a = (float)cos(angle), b = (float)sin(angle);

	//获得图像中心指针
    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
	//获得图像的每行的字节数
    const int step = (int)img.step;

	//原始的BRIEF描述子不具有方向信息，通过加入特征点的方向来计算描述子，称之为Steer BRIEF，具有较好旋转不变特性
	//具体地，在计算的时候需要将这里选取的随机点点集的x轴方向旋转到特征点的方向。
	//获得随机“相对点集”中某个idx所对应的点的灰度,这里旋转前坐标为(x,y), 旋转后坐标(x',y')推导:
    // x'= xcos(θ) - ysin(θ),  y'= xsin(θ) + ycos(θ)
    #define GET_VALUE(idx) center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + cvRound(pattern[idx].x*a - pattern[idx].y*b)]        
    // y'* step
    // x'
	//brief描述子由32*8位组成
	//其中每一位是来自于两个像素点灰度的直接比较，所以每比较出8bit结果，需要16个随机点，这也就是为什么pattern需要+=16的原因
    for (int i = 0; i < 32; ++i, pattern += 16)
    {
		
        int t0, 	//参与比较的一个特征点的灰度值
			t1,		//参与比较的另一个特征点的灰度值	
			val;	//描述子这个字节的比较结果
		
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;							//描述子本字节的bit0
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;					//描述子本字节的bit1
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;					//描述子本字节的bit2
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;					//描述子本字节的bit3
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;					//描述子本字节的bit4
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;					//描述子本字节的bit5
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;					//描述子本字节的bit6
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;					//描述子本字节的bit7

        //保存当前比较的出来的描述子的这个字节
        desc[i] = (uchar)val;
    }//通过对随机点像素灰度的比较，得出BRIEF描述子，一共是32*8=256位

    //为了避免和程序中的其他部分冲突在，在使用完成之后就取消这个宏定义
    #undef GET_VALUE
}
```
## B.对提取的特征点进行矫正
```cpp
void cv::undistortPoints
(
	InputArray _src_//观察点坐标 2xN/Nx2 1-channel or 1xN/Nx1 2-channel (CV_32FC2 or CV_64FC2) (or vector<Point2f> )
	OutputArray _dst_//矫正后的坐标 1xN/Nx1 2-channel or vector<Point2f>
	InputArray _cameraMatrix_ //相机内参
	InputArray _distCoeffs_ //相机畸变矩阵
	InputArray _R_ = noArray //新的变换矩阵
	InputArray _P_ = noArray //新的相机矩阵或者投影矩阵
)
```
## C.计算去畸变后的图像边界

只需在第一帧图像计算一次

```cpp
void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Undistort corners
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}
```
关键点和网格之间映射，后面`GetFeaturesInArea`用到

```cpp
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
```

## D.将特征点分配到网格中

将特征点分配到网格中主要的作用：加速特征点的搜索
* `FRAME_GRID_ROWS`：48
* `FRAME_GRID_COLS`：64
$$
\begin{align}
posX & = \left (x-mnMinX\right )\cdot \frac{ {\small FRAME\_GRID\_COLS}  }{mnMaxX-mnMinX} \\
posY & = \left (y-mnMinY\right )\cdot \frac{ {\small FRAME\_GRID\_ROWS}  }{mnMaxY-mnMinY} 
\end{align}
$$
```cpp
void Frame::AssignFeaturesToGrid()
{
    // Fill matrix with points
    // Step 1  给存储特征点的网格数组 Frame::mGrid 预分配空间
    const int nCells = FRAME_GRID_COLS*FRAME_GRID_ROWS;

    int nReserve = 0.5f*N/(nCells);

    // 开始对mGrid这个二维数组中的每一个vector元素遍历并预分配空间
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
            mGrid[i][j].reserve(nReserve);
            if(Nleft != -1){
                mGridRight[i][j].reserve(nReserve);
            }
        }

    // Step 2 遍历每个特征点，将每个特征点在mvKeysUn中的索引值放到对应的网格mGrid中

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                                 : (i < Nleft) ? mvKeys[i]
                                                                 : mvKeysRight[i - Nleft];
        // 存储某个特征点所在网格的网格坐标，nGridPosX范围：[0,FRAME_GRID_COLS], nGridPosY范围：[0,FRAME_GRID_ROWS]
        int nGridPosX, nGridPosY;
        // 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
        if(PosInGrid(kp,nGridPosX,nGridPosY)){
            if(Nleft == -1 || i < Nleft)
                // 如果找到特征点所在网格坐标，将这个特征点的索引添加到对应网格的数组mGrid中
                mGrid[nGridPosX][nGridPosY].push_back(i);
            else
                mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}
```
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
# 单目初始化
**单目初始化**也就是通过前后两帧图像(`mInitialFrame`与`mCurrentFrame`)进行特征匹配，得到他们之间的**匹配关系**`mvIniMatches`后，通过**H**或者**F**恢复两帧之间的运动，并通过**三角化**生成地图点
![在这里插入图片描述](https://img-blog.csdnimg.cn/b2afd939b78140589b3c98d814e202c7.png)

## 1.接口
```cpp
void Tracking::MonocularInitialization()
```
## 2.步骤
单目初始化的大致步骤如下：
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
## 3.SearchForInitialization
[参考](##SearchForInitialization)

## 4.Reconstruct
在通过**单应性矩阵H**或者**基础矩阵F**进行运动恢复中，采用了**Ransac**方法来估计H或者F
```cpp
        // Step 5 通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated))
```
### 1.RANSAC
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

### 2.归一化
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
### 3.FindFundamental
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
### 4.FindHomography
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
### 5.分数比率
$$
{R}_{H}=\frac{S_H}{S_H+S_F}
$$
* ${R}_{H} > 0.5$ ，选择H矩阵进行运动恢复
* ${R}_{H} \le 0.5$，选择F矩阵进行运动恢复
### 6.重构
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
## 5.CreateInitialMapMonocular
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
# IMU误差
## 1.确定性误差
* Bias：偏置
* Scale：实际数值和传感器输出值之间的比值。
* Misalignment：非正交误差。
* 标定的方法：`六面法`标定`加速度`。
## 2.随机误差
* Allan方差
* 随机游走
# IMU器件测量模型
## 1.角标符号说明
* `b`：body坐标系
* `a`：加速计(acc)
* `g`：陀螺仪(gyro)
* `w`：世界坐标系
* `d`：离散(discrete)
## 2.假设
* 假设：地球是静止的，忽略自转，运行平面为水平面，重力指向固定，模值恒定。
	1. 运行场景小；
	2. 运行时间短；
	3. 精度低(mems)；
## 3.测量模型(gyro/acc)
* 陀螺仪
$$
\tilde{\omega } ^{b}_{wb}\left ( t \right ) = \omega ^{b}_{wb}\left ( t \right ) +b_g\left (  t\right ) +\eta _g\left ( t \right ) 
$$
* 加速计

$$
a^b\left ( t \right ) = {\color{Red} R_{b}^{wT}} \left ( a^{w}-g^{w} \right )  +b_a\left (  t\right ) +\eta _a \left( t \right ) 
$$
* $\widetilde{\omega}, a^b\left ( t \right )$：测量值
* $\omega,a^{w}$：真实值
* `偏置项`和`噪声项`都位于t时刻的`载体坐标系(b)`
* $R_{b}^{w}$：从`载体坐标系(b)`到`世界坐标系(w)`的旋转
* $g^{w}$：`世界坐标系下`的重力加速度

# 预积分
估计的状态：[$R_b^w(t), p^{w}(t), v^{w}(t)$]
* $p^{w}(t)$和$v^{w}(t)$为世界坐标系下IMU的位置和速度
* [$R_b^w(t), p^{w}(t)$]将帧从`B`映射到`W`

## 1.运动方程
使用**欧拉积分**，可以得到**运动方程**的**离散**形式为：
$$
\begin{array}{l}
\mathbf{R}_{b(t+\Delta t)}^{w}=\mathbf{R}_{b(t)}^{w} {\color{Red}\operatorname{Exp} } \left({\color{Red} \boldsymbol{\omega}_{w b}^{b}(t)}  \cdot \Delta t\right) \\
\mathbf{v}^{w}(t+\Delta t)=\mathbf{v}^{w}(t)+{\color{Red} \mathbf{a}^{w}(t)}  \cdot \Delta t \\
\mathbf{p}^{w}(t+\Delta t)=\mathbf{p}^{w}(t)+\mathbf{v}^{w}(t) \cdot \Delta t+{\color{Red} \frac{1}{2} \mathbf{a}^{w}(t)}  \cdot \Delta t^{2}
\end{array}
$$
其中， $w_{wb}^b(t)$表示t时刻`角速度矢量`在b系下的坐标，$w_{wb}^b(t)\cdot \Delta t$表示`旋转矢量`在b系下的坐标，${\color{Red}\operatorname{Exp} } \left({\color{Red} \boldsymbol{\omega}_{w b}^{b}(t)}  \cdot \Delta t\right)$表示在b系下从$t+\Delta t$时刻到$t$时刻的旋转变换($R_{b \left (t + \Delta t \right)}^{b \left ( t \right)}$)。

在采样频率不变，也就是$\Delta t$不变，将**测量模型**代入**离散运动方程**为：
$$
\begin{array}{l}
\mathbf{R}_{k+1}=\mathbf{R}_{k} \cdot \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{k}^{g}-\boldsymbol{\eta}_{k}^{g d}\right) \cdot \Delta t\right) \\
\mathbf{v}_{k+1}=\mathbf{v}_{k}+\mathbf{R}_{k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t+{\color{Red}\mathbf{g} \cdot \Delta t} \\
\mathbf{p}_{k+1}=\mathbf{p}_{k}+\mathbf{v}_{k} \cdot \Delta t+\frac{1}{2} \mathbf{g} \cdot \Delta t^{2}+\frac{1}{2} \mathbf{R}_{k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}
\end{array}
$$
其中，
$$
\mathbf{R}(t) \doteq \mathbf{R}_{b(t)}^{w} ; \boldsymbol{\omega}(t) \doteq \boldsymbol{\omega}_{w b}^{b}(t) ; \mathbf{f}(t)=\mathbf{f}^{b}(t) ; \mathbf{v}(t) \doteq \mathbf{v}^{w}(t) ; \mathbf{p}(t) \doteq \mathbf{p}^{w}(t) ; \mathbf{g} \doteq \mathbf{g}^{w}
$$
## 2.预积分
根据`欧拉积分`，可以利用$i$时刻到$j-1$时刻的**所有IMU测量**，其中$j$时刻的$R_j, v_j, p_j$(`世界坐标系`)可以直接由$i$时刻的$R_i, v_i,p_i$(`世界坐标系`)更新得到。
* $R_j$
$$
R_{j} =R_{i} \prod_{k=i}^{j-1} Exp\left ( \left ( \widetilde{\omega}_{k} -b_k^g-\eta _{k}^{gd}   \right ) \cdot \bigtriangleup t \right ) 
$$

* $v_j$
$$
v_{j} =v_{i} +{\color{Purple} g\cdot \Delta t_{ij}} +\sum_{k=i}^{j-1} {\color{Red} R_k} \cdot \left ( \widetilde{f_k}-b_k^a-\eta _{k}^{ad}  \right )\cdot \Delta  t 
$$
其中，$\left ( \widetilde{f_k}-b_k^a-\eta _{k}^{ad}  \right )\cdot \Delta  t$相对于的是IMU坐标系，`需要转换到世界坐标系`。

* $p_j$
$$
\begin{align}
p_j & = p_i+\sum_{k  = i}^{j-1} v_k\cdot \Delta t+{\color{Purple} \frac{j-i}{2} g\cdot \Delta t^2} +\frac{1}{2} \sum_{k  = i}^{j-1}{\color{Red} R_k} \cdot \left ( \widetilde{f_k}-b_k^a-\eta _k^{ad}  \right )\cdot \Delta t^2 
\\ & = p_i+\sum_{k=i}^{j-1}\left [ v_k\cdot \Delta t+\frac{1}{2} g\cdot \Delta t^2+\frac{1}{2} {\color{Red} R_k} \cdot \left ( \widetilde{f_k}-b_k^a-\eta _k^{ad}  \right )\cdot \Delta t^2 \right ]  
\end{align}
$$
 其中，$\Delta t_{ij}=\sum_{k=i}^{j-1} \Delta t=\left ( j-i \right ) \Delta t$


 为了避免每次更新初始的$R_i, v_i, p_i$都需要重新计算$R_j, v_j, p_j$，引出**预积分项**：
$$
\begin{aligned}
\Delta \mathbf{R}_{i j} & \triangleq \mathbf{R}_{i}^{T} \mathbf{R}_{j} \\
&=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\mathbf{b}_{k}^{g}-\eta_{k}^{g d}\right) \cdot \Delta t\right) \\
\Delta \mathbf{v}_{i j} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right) \\
&=\sum_{k=i}^{j-1} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\eta_{k}^{a d}\right) \cdot \Delta t \\
\Delta \mathbf{p}_{i j} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right) \\
&=\sum_{k=i}^{j-1}\left[\Delta \mathbf{v}_{i k} \cdot \Delta t+\frac{1}{2} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right]
\end{aligned}
$$
### 预积分测量值和测量噪声
假设预积分计算区间内的`bias`相等，即$b_i^g = b_{i+1}^g =...=b_j^{g}$以及$b_i^a = b_{i+1}^a =...=b_j^a$
1. $\Delta R_{ij}$项
根据性质“当$\delta \overrightarrow{\phi }$ ”是`小量`时，有
$$
\operatorname{Exp}(\vec{\phi}+\delta \vec{\phi}) \approx \operatorname{Exp}(\vec{\phi}) \cdot \operatorname{Exp}\left(\mathbf{J}_{r}(\vec{\phi}) \cdot \delta \vec{\phi}\right)
$$
指数映射的Adjoint性质：
$$
\begin{array}{l}
\mathbf{R} \cdot \operatorname{Exp}(\vec{\phi}) \cdot \mathbf{R}^{T} = \exp \left(\mathbf{R} \vec{\phi}^{\wedge} \mathbf{R}^{T}\right) = \operatorname{Exp}(\mathbf{R} \vec{\phi}) \\
\Leftrightarrow \operatorname{Exp}(\vec{\phi}) \cdot \mathbf{R} = \mathbf{R} \cdot \operatorname{Exp}\left(\mathbf{R}^{T} \vec{\phi}\right)
\end{array}
$$
$$
\begin{aligned}
\Delta \mathbf{R}_{i j} &=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\underbrace{\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t}_{\overrightarrow{\phi } }\underbrace{-\boldsymbol{\eta}_{k}^{g d} \Delta t}_{\delta \overrightarrow{\phi } } \right) \\
& \stackrel{(1)}{\approx}  \prod_{k=i}^{j-1}\left\{\operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \cdot \operatorname{Exp}\left(-\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)\right\} \\
& \stackrel{(2)}{=} \Delta \tilde{\mathbf{R}}_{i j} \cdot \prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)
\end{aligned}
$$
其中，令：
$$
\begin{array}{l} 
\mathbf{J}_{r}^{k}=\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)\\
\Delta \tilde{\mathbf{R}}_{i j}=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
\operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)=\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)
\end{array}
$$
其中，$\Delta \widetilde{R}_{jj}=I$，则**可以得到**：
$$
\Delta \mathbf{R}_{i j} \triangleq \Delta \tilde{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)
$$
$\Delta\widetilde{R}_{ij}$即`旋转量预积分测量值`，它由`陀螺仪测量值`和`对陀螺仪bias的估计`得到，$\delta\overrightarrow{\phi }_{ij}$为`测量的噪声`。

2. $\Delta v_{ij}$项：
将$\Delta \mathbf{R}_{i j} \triangleq \Delta \tilde{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)$代入到$\Delta v_{ij}$中
$$
\begin{aligned}
\Delta \mathbf{v}_{i j} &=\sum_{k=i}^{j-1} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t \\
& \approx \sum_{k=i}^{j-1} \Delta \tilde{\mathbf{R}}_{i k} \cdot \underbrace{{\color{Red} \operatorname{Exp}\left(-\delta \vec{\phi}_{i k}\right)}}_{\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}}  \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t \\
& \stackrel{(1)}{\approx} \sum_{k=i}^{j-1} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t \\
& \stackrel{(2)}{\approx} \sum_{k=i}^{j-1}\left[\underbrace{\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t}_{{\color{Red} a^{\wedge } \cdot b = -b^{\wedge }\cdot a} } -\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t +\underline{\Delta\widetilde{R}_{ik}\delta \overrightarrow{\phi} _{ij}^{\wedge }\eta _{k}^{ad}} \right] \\
& \stackrel{(3)}{=} \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t+\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t\right] \\
&=\underbrace{\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right]}_{\Delta \widetilde{v}_{ij} } +\underbrace{\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t\right]}_{-\delta v_{ij}} 
\end{aligned}
$$
**可以得到**：
$$
\Delta v_{ij} \triangleq \Delta \widetilde{v}_{ij}-\delta v_{ij}
$$
$\Delta \widetilde{v}_{ij}$为速度增量预积分测量值，它由IMU的测量值和对bias的估计得到，$\delta v_{ij}$为测量噪声。
3. $\Delta p_{ij}$项
将$\Delta \mathbf{R}_{i j} \triangleq \Delta \tilde{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)$和$\Delta v_{ij} \triangleq \Delta \widetilde{v}_{ij}-\delta v_{ij}$代入到$\Delta p_{ij}$中
$$
\begin{array}{l}
\Delta \mathbf{p}_{i j}=\sum_{k=i}^{j-1}\left[\Delta \mathbf{v}_{i k} \cdot \Delta t+\frac{1}{2} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right] \\
\approx \sum_{k=i}^{j-1}\left[{\color{Red} \left(\Delta \tilde{\mathbf{v}}_{i k}-\delta \mathbf{v}_{i k}\right)}  \cdot \Delta t+\frac{1}{2} {\color{Red} \Delta \tilde{\mathbf{R}}_{i k}  \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i k}\right)} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right] \\
\stackrel{(1)}{\approx} \sum_{k=i}^{j-1}\left[\left(\Delta \tilde{\mathbf{v}}_{i k}-\delta \mathbf{v}_{i k}\right) \cdot \Delta t+\frac{1}{2} \underbrace{\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right)}_{\operatorname{Exp}\left (\vec{ \phi} \right)=I+\vec{\phi}^{\wedge}} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right] \\
\stackrel{(2)}{\approx} \sum_{k=i}^{j-1}\left[\left(\Delta \tilde{\mathbf{V}}_{i k}-\delta \mathbf{v}_{i k}\right) \cdot \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t^{2}-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t^{2} + \underline{\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \delta \vec{\phi}_{i k}^{\wedge} \mathbf{\eta}_{k}^{a d} \Delta t^{2}} \right] \\
\stackrel{(3)}{=} \sum_{k=i}^{j-1}\left[\underbrace{\Delta \tilde{\mathbf{V}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}}_{\Delta \widetilde{p} _{ij}} +\underbrace{\underbrace{\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i k} \Delta t^{2}}_{a^{\wedge }\cdot b=-b^{\wedge }\cdot a} -\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t^{2}-\delta \mathbf{v}_{i k} \Delta t} _{\delta p_{ij}}\right] \\
\end{array}
$$

**可以得到**：
$$
\Delta p_{ij} \triangleq \Delta \widetilde{p}_{ij}-\delta p_{ij}
$$
$\Delta \widetilde{p}_{ij}$为速度增量预积分测量值，它由IMU的测量值和对bias的估计得到，$\delta p_{ij}$为测量噪声。
**预积分理想值**：
$$
\begin{array}{l}
\Delta \mathbf{R}_{i j} \triangleq \mathbf{R}_{i}^{T} \mathbf{R}_{j}\\
\Delta \mathbf{v}_{i j} \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\\
 \Delta \mathbf{p}_{i j} \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)
\end{array}
$$
将预积分理想值代入：**测量值 = 理想值 + 噪声**
$$
\begin{array}{l}
\Delta \tilde{\mathbf{R}}_{i j} \approx \Delta \mathbf{R}_{i j} \operatorname{Exp}\left(\delta \vec{\phi}_{i j}\right)=\mathbf{R}_{i}^{T} \mathbf{R}_{j} \operatorname{Exp}\left(\delta \vec{\phi}_{i j}\right) \\
\Delta \tilde{\mathbf{v}}_{i j} \approx \Delta \mathbf{v}_{i j}+\delta \mathbf{v}_{i j}=\mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)+\delta \mathbf{v}_{i j} \\
\Delta \tilde{\mathbf{p}}_{i j} \approx \Delta \mathbf{p}_{i j}+\delta \mathbf{p}_{i j}=\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)+\delta \mathbf{p}_{i j}
\end{array}
$$
### 噪声分析
1. $\delta \vec{\phi}_{i j}$：
$$
\operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)=\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)
$$
其中，$\mathbf{J}_{r}^{k}=\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)$

对上式两边取对数：
$$
\delta \vec{\phi}_{i j}=-\log \left(\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)\right)
$$
利用**性质**：当$\delta \vec{\phi}$是小量时，$\log{\left(\operatorname{Exp}\left(\vec{\phi} \right) \cdot \operatorname{Exp}\left(\delta\vec{\phi}\right)\right)}=\vec{\phi }+\operatorname{J}^{-1}_{r}\left (\vec{\phi } \right ) \cdot  \delta \vec{\phi }$,  其中$\boldsymbol{\eta}_{k}^{g d}$是小量，$\xi =\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t$是小量，于是$\operatorname{J}^{-1}_{r}\left (\xi_{k} \right )\approx I$

$$
\begin{aligned}
\delta \vec{\phi}_{i j} & =-\log \left(\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& =-\log \left(\operatorname{Exp}\left(-\xi_{i}\right) \prod_{k=i+1}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& \approx-\left(-\xi_{i}+\mathbf{I} \cdot \log \left(\prod_{k=i+1}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right)\right)=\xi_{i}-\log \left(\prod_{k=i+1}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& =\xi_{i}-\log \left(\operatorname{Exp}\left(-\xi_{i+1}\right) \prod_{k=i+2}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& \approx \xi_{i}+\xi_{i+1}-\log \left(\prod_{k=i+2}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& \approx \cdots \\
& \approx \sum_{k=i}^{j-1} \xi_{k}
\end{aligned}
$$
即：
$$
\delta\vec{\phi}_{ij}\approx \sum_{k=i}^{j-1} {\Delta\tilde{R}_{k+1 j}^{T}\operatorname{J}_{r}^{k}\eta_{k}^{gd}\Delta t   }
$$
由于$\Delta\tilde{R}_{k+1 j}^{T}$、$\operatorname{J}_{r}^{k}$、$\Delta t$都是已知量，而$\eta_{k}^{gd}$为零均值高斯噪声，因此$\delta \vec{\phi}_{i j}$也为零均值高斯噪声
2. $\delta v_{ij}$
$$
\delta \mathbf{v}_{i j}=\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t\right]
$$
3. $\delta p_{ij}$
$$
\delta \mathbf{p}_{i j}=\sum_{k=i}^{j-1}\left[\delta \mathbf{v}_{i k} \Delta t-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i k} \Delta t^{2}+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t^{2}\right]
$$
### 噪声更新
1. $\delta\overrightarrow{\phi}_{ij-1}\to  \delta\overrightarrow{\phi}_{ij}$
$$
\begin{aligned}
\delta \vec{\phi}_{i j} & =\sum_{k=i}^{j-1} \Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t \\
& =\sum_{k=i}^{j-2} {\color{Red} \Delta \tilde{\mathbf{R}}_{k+1 j}^{T}}  \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\underbrace{\Delta \tilde{\mathbf{R}}_{j j}^{T}}_{I}  \mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t \\
&= \sum_{j-2}^{k=i}{\color{Red}  \left(\Delta \tilde{\mathbf{R}}_{k+1 j-1} \Delta \tilde{\mathbf{R}}_{j-1 j}\right)^{T}}  \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t \\
& =\Delta \tilde{\mathbf{R}}_{j j-1} \sum_{k=i}^{j-2} \Delta \tilde{\mathbf{R}}_{k+1 j-1}^{T} \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t \\
& =\Delta \tilde{\mathbf{R}}_{j j-1} \delta \vec{\phi}_{i j-1}+\mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t
\end{aligned}
$$
2. $\delta v_{ij-1} \to \delta v_{ij}$
$$
\begin{aligned}
\delta \mathbf{v}_{i j}= & \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t\right] \\
= & \sum_{k=i}^{j-2}\left[\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t\right] \ldots \\
& +\Delta \tilde{\mathbf{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i j-1} \cdot \Delta t \\
= & \delta \mathbf{v}_{i j-1}+\Delta \tilde{\mathbf{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i j-1} \cdot \Delta t
\end{aligned}
$$
3. $\delta p_{ij-1} \to \delta p_{ij}$
$$
\begin{aligned}
\delta \mathbf{p}_{i j} & =\sum_{k=i}^{j-1}\left[\delta \mathbf{v}_{i k} \Delta t-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i k} \Delta t^{2}+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t^{2}\right] \\
& =\delta \mathbf{p}_{i j-1}+\delta \mathbf{v}_{i j-1} \Delta t-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i j-1} \Delta t^{2}+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t^{2}
\end{aligned}
$$
**矩阵C**:
$$
\begin{aligned}
\chi_{i j} & =\left[\eta_{i j}^{T}, \gamma_{i j}^{T}\right]^{T} \sim N\left(0_{15 \times 1}, C_{i j}\right) \\
\eta_{i j} & =\left[\delta \phi_{i j}^{T}, \delta v_{i j}^{T}, \delta p_{i j}^{T}\right]^{T} \sim N\left(0_{9 \times 1}, \Sigma_{i j}^{\eta}\right) \\
\gamma_{i j} & =\left[\delta b_{i j}^{a T}, \delta b_{i j}^{g T}\right]^{T} \sim N\left(0_{6 \times 1}, \Sigma_{i j}^{\gamma}\right)
\end{aligned}
$$
根据$\delta \phi$、$\delta v$、$\delta p$、$\delta b_{a}$、$\delta b_{g}$，可以得到：
$$
C_{i j}=\left(\begin{array}{cc}
\Sigma_{i j}^{\eta} & 0_{9 \times 6} \\
0_{6 \times 9} & \Sigma_{i j}^{\gamma}
\end{array}\right)_{15 \times 15}
$$


$\boldsymbol{\eta}_{i j}^{\Delta} \triangleq\left[\begin{array}{lll}\delta \vec{\phi}_{i j}^{T} & \delta \mathbf{v}_{i j}^{T} & \delta \mathbf{p}_{i j}^{T}\end{array}\right]^{T}$的**递推形式**如下
$$
\begin{aligned}
\boldsymbol{\eta}_{i j}^{\Delta}= &{\underbrace{ {\left[\begin{array}{ccc}
\Delta \tilde{\mathbf{R}}_{j j-1} & \mathbf{0} & \mathbf{0} \\
-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t & \mathbf{I} & \mathbf{0} \\
-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t^{2} & \Delta t \mathbf{I} & \mathbf{I}
\end{array}\right]}}  _{A_{j-1} }\boldsymbol{\eta}_{i j-1}^{\Delta} \cdots } \\
& +\underbrace{\left[\begin{array}{cc}
\mathbf{J}_{r}^{j-1} \Delta t & \mathbf{0} \\
\mathbf{0} & \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t \\
\mathbf{0} & \frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t^{2}
\end{array}\right]}_{B_{j-1}}\underbrace{ \begin{bmatrix}
 \left(\eta_{j-1}^{gd}\right)^T\\
\left(\eta_{j-1}^{ad}\right)^T
\end{bmatrix} } _{\boldsymbol{\eta}_{j-1}^{d}}
\end{aligned}
$$
$\Sigma_{ij}^{\eta}$的**递推形式**如下：
$$
\boldsymbol{\Sigma}_{i j}^{\eta}=\mathbf{A}_{j-1} \boldsymbol{\Sigma}_{i j-1}^{\eta} \mathbf{A}_{j-1}^{T}+\mathbf{B}_{j-1} \boldsymbol{\Sigma}_{\boldsymbol{\eta}} \mathbf{B}_{j-1}^{T}
$$
其中：$\Sigma_{\eta}$为IMU**噪声**的协方差矩阵


$\gamma_{i j} =\left[\delta b_{i j}^{a T}, \delta b_{i j}^{g T}\right]^{T}$的**递推形式**如下
$$
\gamma_{i j}=\gamma_{i j-1}+\sigma_{j-1}^{d}
$$
其中：
$$
 \sigma_{k}^{d}=\left[\left(\sigma_{k}^{b g d}\right)^{T} \left(\sigma_{k}^{b a d}\right)^{T}\right]^{T} 
$$
$\Sigma_{i j}^{\gamma}$的**递推形式**如下：
$$
\begin{array}{l}
 \Sigma_{i j}^{\gamma}=\Sigma_{i j-1}^{\gamma}+\Sigma_{\gamma}
\end{array}
$$
### 预积分测量值更新
当bias不发生变化时
* $\Delta \tilde{R} _{ij-1}\to \Delta \tilde{R}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{R}}_{i j}=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
=\prod_{k=i}^{j-2} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)\cdot \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{j-1}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
=\Delta \tilde{\mathbf{R}}_{i j-1}\cdot\Delta \tilde{\mathbf{R}}_{j-1 j}
\end{array}
$$
* $\Delta \tilde{v}_{ij-1} \to \Delta \tilde{v}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{v}}_{i j} \triangleq \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right] \\
\triangleq \sum_{k=i}^{j-2}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right] + \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\\
\triangleq \Delta \tilde{\mathbf{v}}_{i j-1}+ \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t
\end{array}
$$

* $\Delta \tilde{p}_{ij-1} \to \Delta \tilde{p}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{p}}_{i j} \triangleq \sum_{k = i}^{j-1}\left[\Delta \tilde{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}\right] \\
\triangleq \sum_{k = i}^{j-2}\left[\Delta \tilde{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}\right] + \Delta \tilde{\mathbf{v}}_{i j-1} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \Delta t^{2} \\
\triangleq \Delta \tilde{\mathbf{p}}_{i j-1}  + \Delta \tilde{\mathbf{v}}_{i j-1} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}
\end{array}
$$
### 预积分测量值更新
当bias发生变化时，利用**线性化**来进行bias变化时预积分项的**一阶近似更新**
* bias更新
	* $\bar{b}$：旧的bias
	* $\hat{b}$：新的bias
	* $\delta b$：bias更新量
$$
\begin{array}{c}
\hat{b} _{i}^{g}\gets \bar{b}  _{i}^{g}+\delta b_{i}^{g} \\
\hat{b} _{i}^{a}\gets \bar{b}  _{i}^{a}+\delta b_{i}^{a}
\end{array}
$$
* 一阶近似更新：
$$
\begin{array}{l}
\Delta \tilde{\mathbf{R}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}\right) \approx \Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \\
\Delta \tilde{\mathbf{v}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \approx \Delta \tilde{\mathbf{v}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a} \\
\Delta \tilde{\mathbf{p}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \approx \Delta \tilde{\mathbf{p}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}
\end{array}
$$
* 符号简化：
$$
\begin{array}{l}
\Delta \hat{\mathbf{R}}_{i j} \doteq \Delta \tilde{\mathbf{R}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}\right), \Delta \overline{\mathbf{R}}_{i j} \doteq \Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \\
\Delta \hat{\mathbf{v}}_{i j} \doteq \Delta \tilde{\mathbf{v}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right), \Delta \overline{\mathbf{v}}_{i j} \doteq \Delta \tilde{\mathbf{v}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right) \\
\Delta \hat{\mathbf{p}}_{i j} \doteq \Delta \tilde{\mathbf{p}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right), \Delta \overline{\mathbf{p}}_{i j} \doteq \Delta \tilde{\mathbf{p}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)
\end{array}
$$
* 更新公式可以简化为：
$$
\begin{array}{l}
\Delta \hat{\mathbf{R}}_{i j} \approx \Delta \overline{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \\
\Delta \hat{\mathbf{v}}_{i j} \approx \Delta \overline{\mathbf{v}}_{i j}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a} \\
\Delta \hat{\mathbf{p}}_{i j} \approx \Delta \overline{\mathbf{p}}_{i j}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{aligned}
\Delta \hat{\mathbf{R}}_{i j} & =\Delta \tilde{\mathbf{R}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}\right) \\
& =\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\hat{\mathbf{b}}_{i}^{g}\right) \Delta t\right) \\
& =\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\left(\overline{\mathbf{b}}_{i}^{g}+\delta \mathbf{b}_{i}^{g}\right)\right) \Delta t\right) \\
& =\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\overline{\mathbf{b}}_{i}^{g}\right) \Delta t-\delta \mathbf{b}_{i}^{g} \Delta t\right) \\
& \stackrel{(1)}{\approx}\underbrace{ \prod_{k=i}^{j-1}\left(\operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\overline{\mathbf{b}}_{i}^{g}\right) \Delta t\right) \cdot \operatorname{Exp}\left(-\mathbf{J}_{r}^{k} \delta \mathbf{b}_{i}^{g} \Delta t\right)\right)}_{\operatorname{Exp}(\vec{\phi}+\delta \vec{\phi}) \approx \operatorname{Exp}(\vec{\phi}) \cdot \operatorname{Exp}\left(\mathbf{J}_{r}(\vec{\phi}) \cdot \delta \vec{\phi}\right) and \operatorname{Exp}(\vec{\phi}) \cdot \mathbf{R} = \mathbf{R} \cdot \operatorname{Exp}\left(\mathbf{R}^{T} \vec{\phi}\right)} \\ 
& =\Delta \overline{\mathbf{R}}_{i j} \prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \delta \mathbf{b}_{i}^{g} \Delta t\right)
\end{aligned}
$$
可以得到，
$$
\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}=\sum_{k=i}^{j-1}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
$$
其中，
$$
\mathbf{J}_{r}^{k}=\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)
$$
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$和$\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$
将$\Delta \hat{\mathbf{R}}_{i j}=\Delta \overline{\mathbf{R}}_{i j} \operatorname{Exp}\left(\sum_{k=i}^{j-1}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \delta \mathbf{b}_{i}^{g} \Delta t\right)\right)$代入
$$
\begin{array}{l}
\Delta \hat{\mathbf{v}}_{i j}=\Delta \tilde{\mathbf{v}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \\
=\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right) \Delta t\right] \\
\approx \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right) \Delta t\right] \\
\stackrel{(1)}{\approx} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}+\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right) \Delta t\right] \\
=\sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right) \Delta t-\Delta \overline{\mathbf{R}}_{i k} \delta \mathbf{b}_{i}^{a} \Delta t+\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge}\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right) \Delta t-\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge} \delta \mathbf{b}_{i}^{a} \Delta t\right] \\
\stackrel{(2)}{\approx} \Delta \overline{\mathbf{v}}_{i j}+\sum_{k=i}^{j-1}\left\{-\left[\Delta \overline{\mathbf{R}}_{i k} \Delta t\right] \delta \mathbf{b}_{i}^{a}-\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right] \delta \mathbf{b}_{i}^{g}\right\} \\
\end{array}
$$
**可以得到**：
$$
\begin{aligned}
\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} & =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) \\
\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} & =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right)
\end{aligned}
$$
* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$和 $\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$
$$
\begin{aligned}
\Delta \hat{\mathbf{p}}_{i j} & =\Delta \tilde{\mathbf{p}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \\
& =\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{v}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right) \Delta t^{2}\right] \\
& =\underbrace{\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{v}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \Delta t\right]}_{1}+\underbrace{\frac{1}{2} \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right) \Delta t^{2}\right]}_{2}
\end{aligned}
$$
对于**1**和**2**分别推导：
$$
\begin{aligned}
(1) & =\sum_{k=i}^{j-1}\left[\left(\Delta \overline{\mathbf{v}}_{i k}+\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}\right) \Delta t\right] \\
& =\sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{v}}_{i k} \Delta t+\left(\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) \delta \mathbf{b}_{i}^{g}+\left(\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t\right) \delta \mathbf{b}_{i}^{a}\right]
\end{aligned}
$$
$$
\begin{aligned}
(2) & \approx \frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right)\right] \\
& =\frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right)\right] \\
& \approx \frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}+\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right)\right] \\
& \approx \frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)-\Delta \overline{\mathbf{R}}_{i k} \delta \mathbf{b}_{i}^{a}-\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right]
\end{aligned}
$$
将**1**与**2**组合：
$$
\begin{array}{l}
\Delta \hat{\mathbf{p}}_{i j} =\sum_{k=i}^{j-1}\left\{\left[\Delta \overline{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right) \Delta t^{2}\right]+\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] \delta \mathbf{b}_{i}^{g}+\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] \delta \mathbf{b}_{i}^{a}\right\} \\
=\Delta \overline{\mathbf{p}}_{i j}+\left\{\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right]\right\} \delta \mathbf{b}_{i}^{g}+\left\{\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right]\right\} \delta \mathbf{b}_{i}^{a}
\end{array}
$$
**得到**：
$$
\begin{aligned}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} & =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] \\
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} & =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right]
\end{aligned}
$$
### Jacobian更新
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$
$$
\begin{array}{l}
 \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) \\
=-\sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) - \Delta \overline{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \Delta t \\
= \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}-{\color{Green}  \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Yellow} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} \to \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$ 

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right) \\
 =-\sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right)-\Delta \overline{\mathbf{R}}_{i j-1} \Delta t \\
=\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} -{\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t 
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}  =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] \\
=\sum_{k=i}^{j-2}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Yellow} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} } \Delta t^{2} \\
=\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Yellow} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} } \Delta t^{2}
\end{array}
$$

* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} \to \frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}  =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] \\
 =\sum_{k=i}^{j-2}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} \\
=\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} 
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}=\sum_{k=i}^{j-1}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right) \\
=\sum_{k=i}^{j-2}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
=\sum_{k=i}^{j-2}\left(-\left(\Delta \overline{\mathbf{R}}_{k+1 j-1}\Delta \overline{\mathbf{R}}_{j-1 j} \right)^{T}\mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
=-\Delta \overline{\mathbf{R}}_{ j-1 j}^{T}\cdot \sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{k+1 j-1}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
= -\Delta \overline{\mathbf{R}}_{ j-1 j}^{T}\cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}
-\underbrace{\Delta \overline{\mathbf{R}}_{j j}^{T}}_{I}  \mathbf{J}_{r}^{k} \Delta t
\end{array}
$$
### 残差
* 预积分的测量值采用**一阶近似**修正，免去了积分重新运算，降低计算量
* 状态包含：$\mathbf{R}_{i}, \mathbf{p}_{i}, \mathbf{v}_{i}, \mathbf{R}_{j}, \mathbf{p}_{j}, \mathbf{v}_{j}, \delta \mathbf{b}_{i}^{g}, \delta \mathbf{b}_{i}^{a}$，其中关于bias的是**bias的偏差**
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
#### $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \vec{\phi}_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right) & =\log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right)^{T} \mathbf{R}_{j}\right] \\
& \stackrel{(1)}{=} \log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \operatorname{Exp}\left(-\delta \vec{\phi}_{i}\right) \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right] \\
& \stackrel{2}{=} \log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j} \operatorname{Exp}\left(-\mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i}\right)\right] \\
& =\log \left\{\operatorname{Exp}\left[\log \left(\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right)\right] \cdot \operatorname{Exp}\left(-\mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}\right)\right\} \\
& =\log \left[\operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i}\right)\right) \cdot \operatorname{Exp}\left(-\mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i}\right)\right] \\
& \stackrel{3}{\approx} \mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i}\right)-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i}\right)\right) \mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i} \\
& \stackrel{4}{=} \mathbf{r}_{\Delta \mathbf{R}_{i j}}-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \vec{\phi}_{i}}=-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \mathbf{R}_{j}^{T} \mathbf{R}_{i}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \vec{\phi}_{j}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j} \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right) & =\log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j} \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right] \\
& =\log \left\{\operatorname{Exp}\left[\log \left(\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right)\right] \cdot \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right\} \\
& =\log \left\{\operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j}\right)\right) \cdot \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right\} \\
& \stackrel{(1)}{\approx} \mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j}\right)+\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j}\right)\right) \delta \vec{\phi}_{j} \\
& \stackrel{2}{=} \mathbf{r}_{\Delta \mathbf{R}_{i j}}+\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \delta \vec{\phi}_{j}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \vec{\phi}_{j}}=\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right)
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta p_{i}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{p}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{p}_{i}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta p_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{p}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{p}_{j}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta v_{i}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{v}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{v}_{i}}=\mathbf{0}
$$

* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta v_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{v}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{v}_{j}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \mathbf{b}_{i}^{a}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{b}_{i}^{a}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{b}_{i}^{a}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \mathbf{b}_{i}^{g}$的jacobian
$$
\begin{array}{l}
\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}+\widetilde{\delta \mathbf{b}_{i}^{g}}\right)=\log \left\{\left[\Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}\left(\delta \mathbf{b}_{i}^{g}+\widetilde{\delta \mathbf{b}_{i}^{g}}\right)\right)\right]^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right\} \\
\stackrel{(1)}{\approx} \log \left\{\left[\Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \operatorname{Exp}\left(\mathbf{J}_{r}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right)\right]^{T} \Delta \mathbf{R}_{i j}\right\} \\
\stackrel{(2)}{=} \log \left\{\left[\Delta \hat{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widehat{\delta \mathbf{b}_{i}^{g}}\right)\right]^{T} \Delta \mathbf{R}_{i j}\right\} \\
\stackrel{(3)}{=} \log \left[\operatorname{Exp}\left(-\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right) \Delta \hat{\mathbf{R}}_{i j}^{T} \Delta \mathbf{R}_{i j}\right] \\
=\log \left[\operatorname{Exp}\left(-\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right) \operatorname{Exp}\left(\log \left(\Delta \hat{\mathbf{R}}_{i j}^{T} \Delta \mathbf{R}_{i j}\right)\right)\right] \\
=\log \left[\operatorname{Exp}\left(-\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j} \widetilde{\delta \overline{\mathbf{b}}_{i}^{g}}}{\partial \overline{\mathbf{b}}^{g}}\right) \operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbb{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right)\right] \\
\stackrel{(4)}{=} \log \left\{\operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \operatorname{Exp}\left[-\operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \cdot \varepsilon \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right]\right\} \\
\stackrel{(5)}{\approx} \mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \cdot \operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \cdot \boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}} \\
\stackrel{(6)}{=} \mathbf{r}_{\Delta \mathbf{R}_{i j}}-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbb{R}_{i j}}\right) \cdot \operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \cdot \mathbf{J}_{r}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \cdot \widetilde{\delta \mathbf{b}_{i}^{g}} \\
\end{array}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \widetilde{\delta \mathbf{b}_{i}^{g}}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{b}_{i}^{g}}=-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \cdot \operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \cdot \mathbf{J}_{r}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}
$$
#### $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \vec{\phi}_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right) & =\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right)^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& \stackrel{(1)}{=} \operatorname{Exp}\left(-\delta \overrightarrow{\phi_{i}}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& \stackrel{(2)}{\approx}\left(\mathbf{I}-\left(\delta \vec{\phi}_{i}\right)^{\wedge}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j}-\left(\delta \vec{\phi}_{i}\right)^{\wedge} \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right) \\
& \stackrel{(3)}{=} \mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{R}_{i}\right)+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\right]^{\wedge} \cdot \delta \vec{\phi}_{i} \\
& \stackrel{(4)}{=} \mathbf{r}_{\Delta \mathbf{v}_{i j}}+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\right]^{\wedge} \cdot \delta \vec{\phi}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \vec{\phi}_{i}}=\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\right]^{\wedge}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \vec{\phi}_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta v_{ij}}}{\partial \delta \vec{\phi}_{j}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta p_{i}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{p}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{p}_{i}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta p_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{p}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{p}_{j}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta v_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{i}+\delta \mathbf{v}_{i}\right) & =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\delta \mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j}-\mathbf{R}_{i}^{T} \delta \mathbf{v}_{i} \\
& =\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{i}\right)-\mathbf{R}_{i}^{T} \delta \mathbf{v}_{i} \\
& \stackrel{11}{ }=\mathbf{r}_{\Delta \mathbf{v}_{i j}}-\mathbf{R}_{i}^{T} \delta \mathbf{v}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{v}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{v}_{i}}=-\mathbf{R}_{i}^{T}
$$
*   $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta v_{j}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{j}+\delta \mathbf{v}_{j}\right) & =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}+\delta \mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j}+\mathbf{R}_{i}^{T} \delta \mathbf{v}_{j} \\
& =\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{j}\right)+\mathbf{R}_{i}^{T} \delta \mathbf{v}_{j} \\
& =\mathbf{r}_{\Delta \mathbf{v}_{i j}}+\mathbf{R}_{i}^{T} \delta \mathbf{v}_{j}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{v}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{v}_{j}}=\mathbf{R}_{i}^{T}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \mathbf{b}_{i}^{a}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta v_{i j}}}{\partial \widetilde{\delta \mathbf{b}_{i}^{\mathrm{g}}}}=\frac{\partial \mathbf{r}_{\Delta v_{i j}}}{\partial \delta \mathbf{b}_{i}^{\mathrm{g}}}=-\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \mathbf{b}^{g}}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \mathbf{b}_{i}^{g}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \widetilde{\delta \mathbf{b}_{i}^{\mathrm{a}}}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{b}_{i}^{\mathrm{a}}}=-\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \mathbf{b}^{a}}
$$

#### $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$
*   $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \vec{\phi}_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{\mathrm{p}}}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \overrightarrow{\phi_{i}}\right)\right) & =\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right)^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& \stackrel{(1)}{=} \operatorname{Exp}\left(-\delta \vec{\phi}_{i}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& \stackrel{(2)}{ }\left(\mathbf{I}-\left(\delta \vec{\phi}_{i}\right)^{\wedge}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}-\left(\delta \vec{\phi}_{i}\right)^{\wedge} \mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right) \\
& \stackrel{(3)}{=} \mathbf{r}_{\Delta \mathbf{p}_{y j}}\left(\mathbf{R}_{i}\right)+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)\right]^{\wedge} \cdot \delta \vec{\phi}_{i} \\
& \stackrel{(4)}{=} \mathbf{r}_{\Delta \mathbf{p}_{y}}+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)\right] \cdot \delta \vec{\phi}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \overrightarrow{\phi_{i}}}=\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)\right]^{\wedge}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \vec{\phi}_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \vec{\phi}_{j}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta p_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{i}+\mathbf{R}_{i} \cdot \delta \mathbf{p}_{i}\right) & =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{R}_{i} \cdot \delta \mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}-\mathbf{I} \cdot \delta \mathbf{p}_{i} \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{i}\right)-\mathbf{I} \cdot \delta \mathbf{p}_{i} \\
& \stackrel{1}{ } \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}-\mathbf{I} \cdot \delta \mathbf{p}_{i}
\end{aligned}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta p_{j}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{j}+\mathbf{R}_{j} \cdot \delta \mathbf{p}_{j}\right) & =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}+\mathbf{R}_{j} \cdot \delta \mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}+\mathbf{R}_{i}^{T} \mathbf{R}_{j} \cdot \delta \mathbf{p}_{j} \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{j}\right)+\mathbf{R}_{i}^{T} \mathbf{R}_{j} \cdot \delta \mathbf{p}_{j} \\
& \stackrel{(1)}{=} \mathbf{r}_{\Delta \mathbf{p}_{i j}}+\mathbf{R}_{i}^{T} \mathbf{R}_{j} \cdot \delta \mathbf{p}_{j}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{p}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \mathbf{p}_{j}}=\mathbf{R}_{i}^{T} \mathbf{R}_{j}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta v_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{v}_{i}+\delta \mathbf{v}_{i}\right) & =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\delta \mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}-\mathbf{R}_{i}^{T} \Delta t_{i j} \cdot \delta \mathbf{v}_{i} \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{v}_{i}\right)-\mathbf{R}_{i}^{T} \Delta t_{i j} \cdot \delta \mathbf{v}_{i} \\
& \stackrel{(1)}{ } \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}-\mathbf{R}_{i}^{T} \Delta t_{i j} \cdot \delta \mathbf{v}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{v}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \mathbf{v}_{i}}=-\mathbf{R}_{i}^{T} \Delta t_{i j}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta v_{j}$的jacobian

$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{v}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \mathbf{v}_{j}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \mathbf{b}_{i}^{a}$的jacobian

$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \widehat{\delta \mathbf{b}_{i}^{g}}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{b}_{i}^{g}}=-\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \mathbf{b}^{g}}
$$
* $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \mathbf{b}_{i}^{g}$的jacobian

$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{b}_{i}^{a}}=-\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \mathbf{b}^{a}}
$$
# PreintegrateIMU

这篇博文主要分享**ORB_SLAM3**中`Tracking::PreintegrateIMU()`，其主要包括几个部分：

* 获得两帧之间的IMU数据
* 中值积分
* IMU状态更新

关于IMU的理论推导参考：
* [ORB_SLAM3_IMU预积分理论推导(预积分项)](https://blog.csdn.net/He3he3he/article/details/130047551?spm=1001.2014.3001.5502)
* [ORB_SLAM3_IMU预积分理论推导(噪声分析)](https://blog.csdn.net/He3he3he/article/details/130047460?spm=1001.2014.3001.5502)
* [ORB_SLAM3_IMU预积分理论推导(更新)](https://blog.csdn.net/He3he3he/article/details/130047581?spm=1001.2014.3001.5502)
* [ORB_SLAM3_IMU预积分理论推导(残差)](https://blog.csdn.net/He3he3he/article/details/130054238?spm=1001.2014.3001.5502)
## 1.获得两帧之间的IMU数据
主要是为了获取当前帧和前一帧之间的IMU数据，以便用于后续的IMU预积分
![在这里插入图片描述](https://img-blog.csdnimg.cn/8185005512cc4f5a8164fc553c54d088.png)

```cpp
    while(true)
    {
        // 数据还没有时,会等待一段时间,直到mlQueueImuData中有imu数据.一开始不需要等待
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if(!mlQueueImuData.empty())
            {
                // 拿到第一个imu数据作为起始数据
                IMU::Point* m = &mlQueueImuData.front();
                cout.precision(17);
                // imu起始数据会比当前帧的前一帧时间戳早,如果相差0.001则舍弃这个imu数据
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-mImuPer)
                {
                    mlQueueImuData.pop_front();
                }
                // 同样最后一个的imu数据时间戳也不能理当前帧时间间隔多余0.001
                else if(m->t<mCurrentFrame.mTimeStamp-mImuPer)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    // 得到两帧间的imu数据放入mvImuFromLastFrame中,得到后面预积分的处理数据
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }
```
## 2.中值积分
![在这里插入图片描述](https://img-blog.csdnimg.cn/8c610cc3fe1944b6b0c1a67acad2b177.png)
* 第一帧：线性插值得到$a_{start}$
$$
\frac{1}{2} \left (a_{i}+a_{i+1} - \frac{a_{i+1}-a_{i}}{t_{ab}}\cdot t_{ini} \right )
$$
* 中间帧
$$
\frac{1}{2} \left (a_{i}+a_{i+1} \right )
$$
* 最后帧：线性插值得到$a_{end}$
$$
\frac{1}{2} \left (a_{i}+a_{i+1} - \frac{a_{i+1}-a_{i}}{t_{ab}}\cdot t_{end} \right )
$$

这里讲解下上述公式，以第一帧为例：
* 首先，线性插值得到$a_{start}$，也就是第一个红点处：
$$
\begin{align}
\frac{a_{1}-a_{0}}{t_{1}-t_{0}} & = \frac{a_{0}-a}{t_{0}-t} \\
\Rightarrow a & = a_{0} - \frac{a_{1}-a_{0}}{t_{1}-t_{0}} \cdot \left(t_{0}-t\right)
\end{align}
$$
* 然后，取a与$a_{1}$的平均值，得到

$$
a_{1} + a_{0} - \frac{a_{1}-a_{0}}{t_{1}-t_{0}} \cdot \left(t_{0}-t\right)
$$
```cpp
    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);
    // 针对预积分位置的不同做不同中值积分的处理
    /**
     *  根据上面imu帧的筛选，IMU与图像帧的时序如下：
     *  Frame---IMU0---IMU1---IMU2---IMU3---IMU4---------------IMUx---Frame---IMUx+1
     *  T_------T0-----T1-----T2-----T3-----T4-----------------Tx-----_T------Tx+1
     *  A_------A0-----A1-----A2-----A3-----A4-----------------Ax-----_T------Ax+1
     *  W_------W0-----W1-----W2-----W3-----W4-----------------Wx-----_T------Wx+1
     *  T_和_T分别表示上一图像帧和当前图像帧的时间戳，A(加速度数据)，W(陀螺仪数据)，同理
     */
    for(int i=0; i<n; i++)
    {
        float tstep;
        Eigen::Vector3f acc, angVel;
        // 第一帧数据但不是最后两帧,imu总帧数大于2
        if((i==0) && (i<(n-1)))
        {
            // 获取相邻两段imu的时间间隔
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            // 获取当前imu到上一帧的时间间隔
            float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
            // 设当前时刻imu的加速度a0，下一时刻加速度a1，时间间隔tab 为t10，tini t0p
            // 正常情况下时为了求上一帧到当前时刻imu的一个平均加速度，但是imu时间不会正好落在上一帧的时刻，需要做补偿，要求得a0时刻到上一帧这段时间加速度的改变量
            // 有了这个改变量将其加到a0上之后就可以表示上一帧时的加速度了。其中a0 - (a1-a0)*(tini/tab) 为上一帧时刻的加速度再加上a1 之后除以2就为这段时间的加速度平均值
            // 其中tstep表示a1到上一帧的时间间隔，a0 - (a1-a0)*(tini/tab)这个式子中tini可以是正也可以是负表示时间上的先后，(a1-a0)也是一样，多种情况下这个式子依然成立
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            // 计算过程类似加速度
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            // 中间的数据不存在帧的干扰，正常计算
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        // 直到倒数第二个imu时刻时，计算过程跟第一时刻类似，都需要考虑帧与imu时刻的关系
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
         // 就两个数据时使用第一个时刻的，这种情况应该没有吧，，回头应该试试看
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        // Step 3.依次进行预积分计算
        // 应该是必存在的吧，一个是相对上一关键帧，一个是相对上一帧
        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }
```
## 3.IMU状态更新
在此过程中，视IMU的`bias`不变，更新的顺序如下，主要和每个变量相互之间的依赖关系有关，其依赖关系如图所示
![在这里插入图片描述](https://img-blog.csdnimg.cn/f30ca74bb8c140b08b82813d13f71be0.png)

### 更新[预积分测量值更新](ORB_SLAM3_IMU预积分(理论推导).md#预积分测量值更新)中的**dP**、**dV**
* **dP**包含上一次的**dV**和**dR**
* **dV**包含上一次的**dR**
#### $\Delta \tilde{v}_{ij-1} \to \Delta \tilde{v}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{v}}_{i j} \triangleq \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right] \\
\triangleq \sum_{k=i}^{j-2}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right] + \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\\
\triangleq \Delta \tilde{\mathbf{v}}_{i j-1}+ \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t
\end{array}
$$
#### $\Delta \tilde{p}_{ij-1} \to \Delta \tilde{p}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{p}}_{i j} \triangleq \sum_{k = i}^{j-1}\left[\Delta \tilde{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}\right] \\
\triangleq \sum_{k = i}^{j-2}\left[\Delta \tilde{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}\right] + \Delta \tilde{\mathbf{v}}_{i j-1} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \Delta t^{2} \\
\triangleq \Delta \tilde{\mathbf{p}}_{i j-1}  + \Delta \tilde{\mathbf{v}}_{i j-1} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}
\end{array}
$$
```cpp
    dP = dP + dV * dt + 0.5f * dR * acc * dt * dt;
    dV = dV + dR * acc * dt;
```
### 更新[噪声更新](ORB_SLAM3_IMU预积分(理论推导).md#噪声更新)中的**A**、**B**，主要是与$\Delta \tilde{\mathbf{R}}_{i j-1}$有关部分
$$
\begin{aligned}
\boldsymbol{\eta}_{i j}^{\Delta}= &{\underbrace{ {\left[\begin{array}{ccc}
\Delta \tilde{\mathbf{R}}_{j j-1} & \mathbf{0} & \mathbf{0} \\
-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t & \mathbf{I} & \mathbf{0} \\
-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t^{2} & \Delta t \mathbf{I} & \mathbf{I}
\end{array}\right]}}  _{A_{j-1} }\boldsymbol{\eta}_{i j-1}^{\Delta} \cdots } \\
& +\underbrace{\left[\begin{array}{cc}
\mathbf{J}_{r}^{j-1} \Delta t & \mathbf{0} \\
\mathbf{0} & \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t \\
\mathbf{0} & \frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t^{2}
\end{array}\right]}_{B_{j-1}}\underbrace{ \begin{bmatrix}
 \left(\eta_{j-1}^{gd}\right)^T\\
\left(\eta_{j-1}^{ad}\right)^T
\end{bmatrix} } _{\boldsymbol{\eta}_{j-1}^{d}}
\end{aligned}
$$
```cpp
    Eigen::Matrix<float, 3, 3> Wacc = Sophus::SO3f::hat(acc);

    A.block<3, 3>(3, 0) = -dR * dt * Wacc;
    A.block<3, 3>(6, 0) = -0.5f * dR * dt * dt * Wacc;
    A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<float, 3>(dt, dt, dt);
    B.block<3, 3>(3, 3) = dR * dt;
    B.block<3, 3>(6, 3) = 0.5f * dR * dt * dt;
```

### 更新[Jacobian更新](ORB_SLAM3_IMU预积分(理论推导).md#Jacobian更新)中的**JPa**、**JPg**、**JVa**、**JVg**
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$
$$
\begin{array}{l}
 \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) \\
=-\sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) - \Delta \overline{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \Delta t \\
= \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}-{\color{Green}  \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Purple} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} \to \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$ 

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right) \\
=-\sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right)-\Delta \overline{\mathbf{R}}_{i j-1} \Delta t \\
=\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} -{\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t 
\end{array}
$$

* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}  =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] \\
=\sum_{k=i}^{j-2}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Purple} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} } \Delta t^{2} \\
=\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Purple} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} } \Delta t^{2}
\end{array}
$$

* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} \to \frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}  =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] \\
 =\sum_{k=i}^{j-2}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} \\
=\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} 
\end{array}
$$
```cpp
    JPa = JPa + JVa * dt - 0.5f * dR * dt * dt;
    JPg = JPg + JVg * dt - 0.5f * dR * dt * dt * Wacc * JRg;
    JVa = JVa - dR * dt;
    JVg = JVg - dR * dt * Wacc * JRg;
```

### 更新**dRi**
由罗德里格公式计算$\Delta \tilde{R} _{j-1j}$
```cpp
IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time)
{
    // 得到考虑偏置后的角度旋转
    const float x = (angVel(0) - imuBias.bwx) * time;
    const float y = (angVel(1) - imuBias.bwy) * time;
    const float z = (angVel(2) - imuBias.bwz) * time;

    // 计算旋转矩阵的模值，后面用罗德里格公式计算旋转矩阵时会用到
    const float d2 = x * x + y * y + z * z;
    const float d = sqrt(d2);

    Eigen::Vector3f v;
    v << x, y, z;

    // 角度转成叉积的矩阵形式
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    // eps = 1e-4 是一个小量，根据罗德里格斯公式求极限，后面的高阶小量忽略掉得到此式
    if (d < eps)
    {
        deltaR = Eigen::Matrix3f::Identity() + W;
        rightJ = Eigen::Matrix3f::Identity();
    }
    else
    {
        deltaR = Eigen::Matrix3f::Identity() + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2;
        rightJ = Eigen::Matrix3f::Identity() - W * (1.0f - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
    }
}
```
### 更新[预积分测量值更新](ORB_SLAM3_IMU预积分(理论推导).md#预积分测量值更新)中的**dR**
* $\Delta \tilde{R} _{ij-1}\to \Delta \tilde{R}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{R}}_{i j}=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
=\prod_{k=i}^{j-2} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)\cdot \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{j-1}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
=\Delta \tilde{\mathbf{R}}_{i j-1}\cdot\Delta \tilde{\mathbf{R}}_{j-1 j}
\end{array}
$$
```cpp
dR = NormalizeRotation(dR * dRi.deltaR);
```
### 更新[噪声更新](ORB_SLAM3_IMU预积分(理论推导).md#噪声更新)中的**A**、**B**
$$
\begin{aligned}
\boldsymbol{\eta}_{i j}^{\Delta}= &{\underbrace{ {\left[\begin{array}{ccc}
\Delta \tilde{\mathbf{R}}_{j j-1} & \mathbf{0} & \mathbf{0} \\
-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t & \mathbf{I} & \mathbf{0} \\
-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t^{2} & \Delta t \mathbf{I} & \mathbf{I}
\end{array}\right]}}  _{A_{j-1} }\boldsymbol{\eta}_{i j-1}^{\Delta} \cdots } \\
& +\underbrace{\left[\begin{array}{cc}
\mathbf{J}_{r}^{j-1} \Delta t & \mathbf{0} \\
\mathbf{0} & \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t \\
\mathbf{0} & \frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t^{2}
\end{array}\right]}_{B_{j-1}}\underbrace{ \begin{bmatrix}
 \left(\eta_{j-1}^{gd}\right)^T\\
\left(\eta_{j-1}^{ad}\right)^T
\end{bmatrix} } _{\boldsymbol{\eta}_{j-1}^{d}}
\end{aligned}
$$
```cpp
    A.block<3, 3>(0, 0) = dRi.deltaR.transpose();
    B.block<3, 3>(0, 0) = dRi.rightJ * dt;
```
### 更新[Jacobian更新](ORB_SLAM3_IMU预积分(理论推导).md#Jacobian更新)中的**JRg**
* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} \to \frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}  =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] \\
 =\sum_{k=i}^{j-2}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} \\
=\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} 
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}=\sum_{k=i}^{j-1}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right) \\
=\sum_{k=i}^{j-2}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
=\sum_{k=i}^{j-2}\left(-\left(\Delta \overline{\mathbf{R}}_{k+1 j-1}\Delta \overline{\mathbf{R}}_{j-1 j} \right)^{T}\mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
=-\Delta \overline{\mathbf{R}}_{ j-1 j}^{T}\cdot \sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{k+1 j-1}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
= -\Delta \overline{\mathbf{R}}_{ j-1 j}^{T}\cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}
-\underbrace{\Delta \overline{\mathbf{R}}_{j j}^{T}}_{I}  \mathbf{J}_{r}^{k} \Delta t
\end{array}
$$
```cpp
JRg = dRi.deltaR.transpose() * JRg - dRi.rightJ * dt;
```
## 完整代码
```cpp
void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt)
{
    // 保存imu数据，利用中值积分的结果构造一个预积分类保存在mvMeasurements中
    mvMeasurements.push_back(integrable(acceleration, angVel, dt));

    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.

    // Matrices to compute covariance
    // Step 1.构造协方差矩阵
    // 噪声矩阵的传递矩阵，这部分用于计算i到j-1历史噪声或者协方差
    Eigen::Matrix<float, 9, 9> A;
    A.setIdentity();
    // 噪声矩阵的传递矩阵，这部分用于计算j-1新的噪声或协方差，这两个矩阵里面的数都是当前时刻的，计算主要是为了下一时刻使用
    Eigen::Matrix<float, 9, 6> B;
    B.setZero();

    // 考虑偏置后的加速度、角速度
    Eigen::Vector3f acc, accW;
    acc << acceleration(0) - b.bax, acceleration(1) - b.bay, acceleration(2) - b.baz;
    accW << angVel(0) - b.bwx, angVel(1) - b.bwy, angVel(2) - b.bwz;

    // 记录平均加速度和角速度
    avgA = (dT * avgA + dR * acc * dt) / (dT + dt);
    avgW = (dT * avgW + accW * dt) / (dT + dt);

    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    // 根据没有更新的dR来更新dP与dV  eq.(38)
    dP = dP + dV * dt + 0.5f * dR * acc * dt * dt;
    dV = dV + dR * acc * dt;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    // 根据η_ij = A * η_i,j-1 + B_j-1 * η_j-1中的Ａ矩阵和Ｂ矩阵对速度和位移进行更新
    Eigen::Matrix<float, 3, 3> Wacc = Sophus::SO3f::hat(acc);

    A.block<3, 3>(3, 0) = -dR * dt * Wacc;
    A.block<3, 3>(6, 0) = -0.5f * dR * dt * dt * Wacc;
    A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<float, 3>(dt, dt, dt);
    B.block<3, 3>(3, 3) = dR * dt;
    B.block<3, 3>(6, 3) = 0.5f * dR * dt * dt;

    // Update position and velocity jacobians wrt bias correction
    // 因为随着时间推移，不可能每次都重新计算雅克比矩阵，所以需要做J(k+1) = j(k) + (~)这类事，分解方式与AB矩阵相同
    // 论文作者对forster论文公式的基础上做了变形，然后递归更新，参见 https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/212
    JPa = JPa + JVa * dt - 0.5f * dR * dt * dt;
    JPg = JPg + JVg * dt - 0.5f * dR * dt * dt * Wacc * JRg;
    JVa = JVa - dR * dt;
    JVg = JVg - dR * dt * Wacc * JRg;

    // Update delta rotation
    // Step 2. 构造函数，会根据更新后的bias进行角度积分
    IntegratedRotation dRi(angVel, b, dt);
    // 强行归一化使其符合旋转矩阵的格式
    dR = NormalizeRotation(dR * dRi.deltaR);

    // Compute rotation parts of matrices A and B
    // 补充AB矩阵
    A.block<3, 3>(0, 0) = dRi.deltaR.transpose();
    B.block<3, 3>(0, 0) = dRi.rightJ * dt;

    // 小量delta初始为0，更新后通常也为0，故省略了小量的更新
    // Update covariance
    // Step 3.更新协方差，frost经典预积分论文的第63个公式，推导了噪声（ηa, ηg）对dR dV dP 的影响
    C.block<9, 9>(0, 0) = A * C.block<9, 9>(0, 0) * A.transpose() + B * Nga * B.transpose();  // B矩阵为9*6矩阵 Nga 6*6对角矩阵，3个陀螺仪噪声的平方，3个加速度计噪声的平方
    // 这一部分最开始是0矩阵，随着积分次数增加，每次都加上随机游走，偏置的信息矩阵
    C.block<6, 6>(9, 9) += NgaWalk;

    // Update rotation jacobian wrt bias correction
    // 计算偏置的雅克比矩阵，r对bg的导数，∂ΔRij/∂bg = (ΔRjj-1) * ∂ΔRij-1/∂bg - Jr(j-1)*t
    // 论文作者对forster论文公式的基础上做了变形，然后递归更新，参见 https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/212
    // ? 为什么先更新JPa、JPg、JVa、JVg最后更新JRg? 答：这里必须先更新dRi才能更新到这个值，但是为什么JPg和JVg依赖的上一个JRg值进行更新的？
    JRg = dRi.deltaR.transpose() * JRg - dRi.rightJ * dt;

    // Total integrated time
    // 更新总时间
    dT += dt;
}
```
# TrackWithMotionModel
## 1.更新上一帧位姿
`Tracking::UpdateLastFrame()`的主要作用是**更新上一帧的位姿**和**添加一些临时的地图点**，为什么要更新上一帧的位姿，主要是在ORB_SLAM中优化的是**参考关键帧**的位姿，对于**普通帧**，虽然在开始设置了位姿，但是没有参与优化，因此在下一次跟踪时，需要用**优化后的参考关键帧**的位姿更新上一帧的位姿
* `mlRelativeFramePoses`存储**参考关键帧**`r`到**当前帧**`c`的位姿$T_{cr}$
$$
T_{cr} = T_{cw} \cdot T_{rw}^{-1}
$$

* 利用参考关键帧更新上一帧在世界坐标系下的**位姿**
$$
T_{lw} = T_{lr}\cdot T_{rw}
$$
```cpp
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());
```
对于**双目**或**RGBD**，为上一帧生成新的**临时地图点，主要是为了生成更多的匹配，让跟踪更好**
* 临时地图点：对于**上一帧**中具有有效深度值`z>0`的特征点，如果这个**特征点**在**上一帧**中**没有**对应的地图点，或者创建后**没有被观测到**，添加为临时地图点
  * 跟踪OK后，在创建新的关键帧前将临时地图点`mlpTemporalPoints`全部删除
* 临时地图点也不是越多越好，当满足下面两个条件停止添加：
	* 当前的点的深度已经超过了设定的深度阈值(**35倍基线**)，主要太远了不可靠
	* **具有有效深度的点**已超过`100`个点，说明距离比较远了，可能不准确，这里是从近的开始添加

```cpp

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for(int i=0; i<Nfeat;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    // 按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;
        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
            bCreateNew = true;

        if(bCreateNew)
        {
            Eigen::Vector3f x3D;

            if(mLastFrame.Nleft == -1){
                mLastFrame.UnprojectStereo(i, x3D);
            }
            else{
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
            mLastFrame.mvpMapPoints[i]=pNewMP;
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;

    }
```
## 2.得到当前帧的初始位姿
如果IMU已初始化并且不需要`reset`时，使用`PredictStateIMU`来预测当前帧的状态，就不用通过**匀速模型**来得到了
### PredictStateIMU
这里有两个变量控制着从哪预测
* `mbMapUpdated`：地图是否更新
* `mpLastKeyFrame`：上一关键帧存在

于是有两种情况：
* 如果地图更新了，且**上一关键帧**存在，则用关键帧来进行预测`mpImuPreintegratedFromLastKF`
* 如果地图未更新，则用**上一帧**来进行预测`mpImuPreintegratedFrame`


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
### 匀速模型
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
```cpp
mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
```



然后，基于**投影的匹配搜索**[SearchByProjection](##SearchByProjection(TrackWithMotionModel))获得上一帧与当前帧的匹配关系，如果匹配太少`<20`，则会**扩大搜索窗口**

```cpp
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;

    int nmatches = matcher.SearchByProjection(
        mCurrentFrame,
        mLastFrame,
        th,
        mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR
    );

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(
            mCurrentFrame.mvpMapPoints.begin(),
            mCurrentFrame.mvpMapPoints.end(),
            static_cast<MapPoint*>(NULL)
        );

        nmatches = matcher.SearchByProjection(
            mCurrentFrame,
            mLastFrame,
            2*th,
            mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR
        );
    }
```



## 3.位姿优化
得到上一帧与当前帧的匹配关系后，利用`3D-2D`投影关系优化当前帧位姿[PoseOptimization](##PoseOptimization)

```cpp
Optimizer::PoseOptimization(&mCurrentFrame);
```



## 4.剔除当前帧中地图点中的外点
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

# TrackReferenceKeyFrame
* 使用条件：
  1. **运动模型为空**并且**imu未初始化**，说明是**刚初始化完**第一帧跟踪，或者已经跟丢了
  2. **当前帧**和**重定位帧**间隔**很近**，用**重定位帧**来恢复位姿
  3. **恒速云端模型**跟踪失败
## 1.计算当前帧的描述子的Bow向量
```cpp
mCurrentFrame.ComputeBoW();
```
`DBoW2`由一棵词汇树(`Vocabulary Tree`)、逆向索引表(`Inverse Indexes`)以及一个正向索引表(`Direct Indexes`)三个部分构成。对于词汇树，它是由一堆图像离线训练而来的，首先对训练的图像计算`ORB`特征点，然后把这些特征点放在一起，通过`K-means`对它们聚类， 将之分为`k`类。然后对每簇，再次通过`K-means`进行聚类。如此重复`L`次，就得到了深度为`L`的树，除了叶子之外，每个节点都有`k`个子节点。 

`ComputeBoW`主要是通过`transform`函数将当前帧的描述子`mDescriptors`转换为`mBowVec`和`mFeatVec`，其中：
* `mBowVec`：[单词的`Id`，权重]
	* `std::map<WordId, WordValue>`
	* 单词的`Id`：为词汇树中距离最近的叶子节点的id
	* 权重：对于同一个单词，权重累加
* `mFeatVec`：[Node的`Id`，对应的图像`feature`的`Ids`]
	* `std::map<NodeId, std::vector<unsigned int> >` 
	* Node的`Id`：距离叶子节点深度为level up对应的node的`Id`
	* 对应的图像`feature`的`Id`：该节点下**所有叶子节点**对应的`feature`的`id`
* `level up`确定搜素范围：
	* 如果 `level up`越大，那么`featureVec`的`size`越大，搜索的范围越广，速度越慢；
	* 如果`level up`越小，那么`featureVec`的`size`越小，搜索的范围越小，速度越快。


![在这里插入图片描述](https://img-blog.csdnimg.cn/95d0ba64e3f4427995f5753452a45751.png)
```cpp
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(const TDescriptor &feature, 
  WordId &word_id, WordValue &weight, NodeId *nid, int levelsup) const
{ 
  // propagate the feature down the tree
  vector<NodeId> nodes;
  typename vector<NodeId>::const_iterator nit;

  // level at which the node must be stored in nid, if given
  const int nid_level = m_L - levelsup;
  if(nid_level <= 0 && nid != NULL) *nid = 0; // root

  NodeId final_id = 0; // root
  int current_level = 0;

  do
  {
    ++current_level;
    nodes = m_nodes[final_id].children;
    final_id = nodes[0];
 
    double best_d = F::distance(feature, m_nodes[final_id].descriptor);

    for(nit = nodes.begin() + 1; nit != nodes.end(); ++nit)
    {
      NodeId id = *nit;
      double d = F::distance(feature, m_nodes[id].descriptor);
      if(d < best_d)
      {
        best_d = d;
        final_id = id;
      }
    }
    
    if(nid != NULL && current_level == nid_level)
      *nid = final_id;
    
  } while( !m_nodes[final_id].isLeaf() );

  // turn node id into word id
  word_id = m_nodes[final_id].word_id;
  weight = m_nodes[final_id].weight;
}
```
## 2.特征匹配

通过[SearchByBoW](##SearchByBow(Tracking))加速**当前帧**与**参考帧**之间的特征匹配

```cpp
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
```

## 3.将上一帧的位姿作为当前帧的位姿的初值
```cpp
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());  
```
## 4.位姿优化

通过[PoseOptimization](##PoseOptimization)最小化重投影误差优化当前帧位姿

```cpp
Optimizer::PoseOptimization(&mCurrentFrame);
```
## 5.剔除优化后匹配中的外点
```cpp
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            // 如果对应到的某个特征点是外点
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                // 匹配的内点计数++
                nmatchesMap++;
        }
    }
```
# TrackLocalMap
通过[TrackWithMotionModel](#TrackWithMotionModel)或[TrackReferenceKeyFrame](#TrackReferenceKeyFrame)的`Frame to Frame`跟踪得到了**当前帧的初始位姿**。为了得到更加精准的位姿，可以将**局部地图**投影到当前帧上得到更多的匹配实现`Frame to Map`，然后进一步优化当前帧的位姿
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
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }
```
2. 对于局部地图中的地图点，判断是否在当前帧的视野范围内
```cpp

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
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            // 观测到该点的帧数加1
            pMP->IncreaseVisible();
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

3  当前帧视野范围内的地图点投影到当前帧，通过[SearchByProjection](##SearchByProjection(TrackLocalMap))得到更多的匹配

```cpp
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
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
## 3.位姿优化
优化分为两种情况：
* IMU未初始化 或者 初始化了但是刚刚重定位：[PoseOptimization](##PoseOptimization)
* 其他情况：[PoseInertialOptimizationLastFrame](##PoseInertialOptimizationLastFrame)或者`PoseInertialOptimizationLastKeyFrame`
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
# Relocalization
`Relocalization`主要的作用是在**跟踪失败**时，通过词袋在关键帧数据库`KeyFrameDatabase`中寻找和当前帧相似的关键帧作为匹配帧，进而恢复当前帧的位姿

1. 计算当前帧的`Bow`，参考[Bow向量](##1.计算当前帧的描述子的Bow向量)
```cpp
mCurrentFrame.ComputeBoW();
```
2. 检测当前帧的重定位候选帧`vpCandidateKFs`
```cpp
vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());
```
3. 对于所有的候选关键帧，通过[SearchByBoW](##)搜索与当前帧的匹配，并初始化对应的`MLPnPsolver`
```cpp
    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    // 每个关键帧的解算器
    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    // 每个关键帧和当前帧中特征点的匹配关系
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    // 放弃某个关键帧的标记
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    // 有效的候选关键帧数目
    int nCandidates=0;

    // Step 3：遍历所有的候选关键帧，通过BoW进行快速匹配，用匹配结果初始化PnP Solver
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 当前帧和候选关键帧用BoW进行快速匹配，匹配结果记录在vvpMapPointMatches，nmatches表示匹配的数目
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            // 如果和当前帧的匹配数小于15,那么只能放弃这个关键帧
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // 如果匹配数目够用，用匹配结果初始化MLPnPsolver
                // ? 为什么用MLPnP? 因为考虑了鱼眼相机模型，解耦某些关系？
                // 参考论文《MLPNP-A REAL-TIME MAXIMUM LIKELIHOOD SOLUTION TO THE PERSPECTIVE-N-POINT PROBLEM》
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                // 构造函数调用了一遍，这里重新设置参数
                pSolver->SetRansacParameters(
                    0.99,                    // 模型最大概率值，默认0.9
                    10,                      // 内点的最小阈值，默认8
                    300,                     // 最大迭代次数，默认300
                    6,                       // 最小集，每次采样六个点，即最小集应该设置为6，论文里面写着I > 5
                    0.5,                     // 理论最少内点个数，这里是按照总数的比例计算，所以epsilon是比例，默认是0.4
                    5.991);                  // 卡方检验阈值 //This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;  // 1.0版本新加的
            }
        }
    }
```
4. 从候选关键帧中找到能够匹配上的关键帧，当位姿优化`PoseOptimization`的内点数到达`50`则成功
![在这里插入图片描述](https://img-blog.csdnimg.cn/0c89abd32e104e9f99435694cc58f06f.png)

PnP+Opt：
* 通过`MLPnPsolver`计算**当前帧**的位姿`eigTcw`，并设置当前帧位姿的初值
* 根据`PnP`的内外点`vbInliers`更新下当前帧的地图点，然后用`PoseOptimization`优化当前帧的Pose
* 根据优化后的内外点`mCurrentFrame.mvbOutlier`更新当前帧的地图点
* 如果内点太少`10`，直接跳过；如果内点大于`50`，匹配成功；如果内点在`[10, 50]`之间，可以再尝试下
```cpp
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;
			//PnP
            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(bTcw)
            {
            	//设置当前帧的位姿的初值
                Sophus::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                set<MapPoint*> sFound;

                const int np = vbInliers.size();
				//更新当前帧的地图点
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }
				//位姿优化
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
				//内点太少了，直接退出
                if(nGood<10)
                    continue;
				//根据位姿优化后的内点，更新当前帧的地图点
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);
              .....
              }
```
注意：`sFound`赋值是在PnP后，而不在优化后，说明`sFound`为包含了PnP的所有内点的集合。为啥不在优化后赋值，主要是为了防止通过`SearchByProjection`匹配搜索到**新的匹配**为优化后的外点，那么再位姿优化，内点可能无法达标。

第一次尝试：
* 因为内点数**不够**`50`，那么尝试通过`SearchByProjection`将关键帧中**不属于当前帧**的地图点投影到当前帧中获得一些新的匹配，新的匹配数为`nadditional`。
* 如果**新的匹配+内点数**大于50，那么可以再用`PoseOptimization`优化当前帧的位姿试试
* 这时候判断下是否成功，如果优化后的内点数仍不足`50`而在`[30, 50]`之间，那么说明离匹配不远可以再试试
```cpp
  if(nGood<50)
  {
      int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);
      if(nadditional+nGood>=50)
      {
          nGood = Optimizer::PoseOptimization(&mCurrentFrame);
       }
  }
```
第二次尝试
* 因为当前帧的位姿已经优化了很多次，而内点离`50`很近了，说明当前帧的位姿更准，所以缩小搜索窗口，通过`SearchByProjection`在**更狭窄的窗口**上进行搜索得到额外的匹配点数。
* 如果**新的匹配+内点数**大于50，那么可以再用`PoseOptimization`优化当前帧的位姿试试
* 如果内点大于`50`，那么匹配成功


```cpp
   if(nGood>30 && nGood<50)
   {
       // 用更小窗口、更严格的描述子阈值，重新进行投影搜索匹配
       sFound.clear();
       for(int ip =0; ip<mCurrentFrame.N; ip++)
           if(mCurrentFrame.mvpMapPoints[ip])
               sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
       nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

       // Final optimization
       if(nGood+nadditional>=50)
       {
           nGood = Optimizer::PoseOptimization(&mCurrentFrame);
           for(int io =0; io<mCurrentFrame.N; io++)
               if(mCurrentFrame.mvbOutlier[io])
                   mCurrentFrame.mvpMapPoints[io]=NULL;
       }
   }
   .....
```
注意：
* `sFound`包含了第一次尝试时当前帧的内点以及额外搜索到的新的匹配
* 第一尝试优化后没有利用内点更新当前帧的地图点，即使匹配成功，而第二次则有（**不太合理**）
## DetectRelocalizationCandidates
`DetectRelocalizationCandidates`的作用是通过词袋模糊搜索在**已有的关键帧**中查找和**当前帧**最接近的候选帧，其函数接口如下：
```cpp
vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, Map *pMap)
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/073b84486ef547768207290265578319.png)

1. 遍历当前帧中所有的`mBowVec`，每个word由`WordId`和`WordValue`组成，利用倒排索引`mvInvertedFile`得到拥有相同`Bow`的关键帧，并统计相同单词的数量
```cpp
    list<KeyFrame *> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                if (pKFi->mnRelocQuery != F->mnId)
                {
                    pKFi->mnRelocWords = 0;
                    pKFi->mnRelocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if (lKFsSharingWords.empty())
        return vector<KeyFrame *>();
```
2. 获得最大最小公共单词数，得到：$minCommonWords = maxCommonWords \cdot 0.8f$
```cpp
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        if ((*lit)->mnRelocWords > maxCommonWords)
            maxCommonWords = (*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;
```
3. 遍历所有具有公共单词的关键帧， 过滤掉公共单词数**小于**`minCommonWords`的关键帧，并计算关键帧与当前帧`mBowVec`的相似性`score`
```cpp
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;

        if (pKFi->mnRelocWords > minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
            pKFi->mRelocScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }
```
4. 每个关键帧和最佳的10个共视关键帧组队，得到每组中分数最大的关键帧和组的总分数
```cpp
    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
    {
        KeyFrame *pKFi = it->second;
        vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame *pBestKF = pKFi;
        for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
        {
            KeyFrame *pKF2 = *vit;
            if (pKF2->mnRelocQuery != F->mnId)
                continue;

            accScore += pKF2->mRelocScore;
            if (pKF2->mRelocScore > bestScore)
            {
                pBestKF = pKF2;
                bestScore = pKF2->mRelocScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }
```
5. 过滤掉组的总分数不合格的关键帧，得到最终的结果
```cpp
    float minScoreToRetain = 0.75f * bestAccScore;
    set<KeyFrame *> spAlreadyAddedKF;
    vector<KeyFrame *> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
    {
        const float &si = it->first;
        if (si > minScoreToRetain)
        {
            KeyFrame *pKFi = it->second;
            if (pKFi->GetMap() != pMap)
                continue;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
```
# 匹配
## SearchForInitialization
`SearchForInitialization`主要的作用是寻找**初始帧F1**与**当前帧F2**之间的匹配关系`vnMatches12`，其输入参数如下：

|  参数                                                                                                                                                                                                |  描述            |
|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:---------------|
| F1                                                                                                                                                                                               | 初始帧            |
| F2                                                                                                                                                                                                 | 当前帧            |
| vbPrevMatched                                                                                                                                                                                      | 初始帧中特征点的坐标     |
| vnMatches12                                                                                                                                                                                        | 匹配关系           |
| windowSize                                                                                                                                                                                         | 搜索窗口大小         |  

其组件包括：搜索窗口、重复匹配过滤、最佳描述子距离、最优与次优比值、旋转直方图

```cpp
int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
```

### 1.旋转直方图的构建
```cpp
		vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;
```
### 2.搜索候选匹配点
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
            vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);
```
### 3.通过描述子的距离搜索候选匹配点中的最优与次优
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
### 4.条件筛选
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
### 5.旋转直方图
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
### 6.更新匹配
```cpp
        // Step 7 将最后通过筛选的匹配好的特征点保存到vbPrevMatched
        for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
            if(vnMatches12[i1]>=0)
                vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;
```
## SearchByProjection(TrackWithMotionModel)

`SearchByProjection`函数接口：

```cpp
int ORBmatcher::SearchByProjection(
    Frame &CurrentFrame,
    const Frame &LastFrame,
    const float th,
    const bool bMono
)
```

### 1.旋转直方图的构建

```cpp
vector<int> rotHist[HISTO_LENGTH];
for(int i=0;i<HISTO_LENGTH;i++)
    rotHist[i].reserve(500);
const float factor = 1.0f/HISTO_LENGTH;
```
### 2.计算当前帧和前一帧的平移$t_{lc}$，判断相机是前进还是后退

* 近大远小，尺度越大，图像越小

* 前进：$z > b$，物体在当前帧的图像上**变大**，因此对于上一帧的特征点，需要在当前帧**更高**的尺度上搜索
* 后退：$z < -b$，物体在当前帧的图像上**变小**，因此对于上一帧的特征点，需要在当前帧**更低**的尺度上搜索

```cpp
        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        const Eigen::Vector3f twc = Tcw.inverse().translation();

        const Sophus::SE3f Tlw = LastFrame.GetPose();
        const Eigen::Vector3f tlc = Tlw * twc;

        const bool bForward = tlc(2)>CurrentFrame.mb && !bMono;
        const bool bBackward = -tlc(2)>CurrentFrame.mb && !bMono;
```
### 3.对于前一帧的每一个地图点，通过相机投影模型，投影到当前帧

```cpp
        Eigen::Vector3f x3Dw = pMP->GetWorldPos();
        Eigen::Vector3f x3Dc = Tcw * x3Dw;

        const float xc = x3Dc(0);
        const float yc = x3Dc(1);
        const float invzc = 1.0/x3Dc(2);

        if(invzc<0)
            continue;

        Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/10bf7c4ba9c44055a9accc0e6fb45565.png)

### 4.根据相机的前进后退方向来判断搜索尺度范围

* 前进：$z > b$，物体在当前帧的图像上**变大**，因此对于上一帧的特征点，需要在当前帧**更高**的尺度上搜索
* 后退：$z < -b$，物体在当前帧的图像上**变小**，因此对于上一帧的特征点，需要在当前帧**更低**的尺度上搜索
![在这里插入图片描述](https://img-blog.csdnimg.cn/be2af519b6c449a09446d30c3385d268.png)


```cpp
    if(uv(0)<CurrentFrame.mnMinX || uv(0)>CurrentFrame.mnMaxX)
        continue;
    if(uv(1)<CurrentFrame.mnMinY || uv(1)>CurrentFrame.mnMaxY)
        continue;

    int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? 	
        LastFrame.mvKeys[i].octave : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

    // Search in a window. Size depends on scale
    float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

    vector<size_t> vIndices2;

    if(bForward)
        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave);
    else if(bBackward)
        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, 0, nLastOctave);
    else
        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0),uv(1), radius, nLastOctave-1, nLastOctave+1);

    if(vIndices2.empty())
        continue;
```
### 5.遍历候选匹配点，寻找距离最小的最佳匹配点
这里就简单选了个最佳匹配点，其他的像剔除重复匹配，最佳和次佳比都没做

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
### 6.计算匹配点对的**旋转角度差**所在的直方图
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
                                                    : (i < LastFrame.Nleft) ? 		LastFrame.mvKeys[i]
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
### 7.进行旋转一致检测，剔除不一致的匹配
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
## SearchByProjection(TrackLocalMap)
`SearchByProjection`通过将**局部地图中在当前帧视野范围内的地图点**投影到**当前帧**寻找更多的匹配关系，其函数接口：

```cpp
int ORBmatcher::SearchByProjection(
	Frame &F,
	const vector<MapPoint*> &vpMapPoints,
	const float th,
	const bool bFarPoints,
	const float thFarPoints
)
```
### 1.对当前帧视野范围内的地图点`mbTrackInView = true`进行投影
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

### 2.通过地图点的投影获得当前帧的候选匹配点
若视角夹角较小时, 搜索半径较小
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
### 3.在候选匹配点中，搜索地图点的最佳与次佳匹配点
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
### 4.条件筛选匹配
条件：
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
## SearchByProjection(Relocalization)
`SearchByProjection`的主要作用是通过将关键帧中的地图点投影到当前帧寻找两帧之间**新的匹配**，关键帧中的地图点不在`sAlreadyFound`中
1. 构造旋转直方图
```cpp
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;
```
2. 将关键帧中的地图点投影到当前帧寻找匹配，需要满足如下要求：
	* 该地图点不在`sAlreadyFound`中
	* 深度$\mathbf{dist3D}=\mathbf{norm}\left(X_w-O_w\right)$ 必须在$[\mathbf{minDistance}, \mathbf{maxDistance}]$之内
	* 根据地图点在当前帧预测的尺度搜索候选匹配点
	* 最佳匹配距离小于阈值`ORBdist`
	* 计算旋转直方图
```cpp
        for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMPs[i];

            if(pMP)
            {
                if(!pMP->isBad() && !sAlreadyFound.count(pMP))
                {
                    //Project
                    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                    Eigen::Vector3f x3Dc = Tcw * x3Dw;

                    const Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                    if(uv(0)<CurrentFrame.mnMinX || uv(0)>CurrentFrame.mnMaxX)
                        continue;
                    if(uv(1)<CurrentFrame.mnMinY || uv(1)>CurrentFrame.mnMaxY)
                        continue;

                    // Compute predicted scale level
                    Eigen::Vector3f PO = x3Dw-Ow;
                    float dist3D = PO.norm();

                    const float maxDistance = pMP->GetMaxDistanceInvariance();
                    const float minDistance = pMP->GetMinDistanceInvariance();

                    // Depth must be inside the scale pyramid of the image
                    if(dist3D<minDistance || dist3D>maxDistance)
                        continue;

                    //预测尺度
                    int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                    // Search in a window
                    const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                    //  Step 3 搜索候选匹配点
                    const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nPredictedLevel-1, nPredictedLevel+1);

                    if(vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;
                    // Step 4 遍历候选匹配点，寻找距离最小的最佳匹配点 
                    for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                    {
                        const size_t i2 = *vit;
                        if(CurrentFrame.mvpMapPoints[i2])
                            continue;

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP,d);

                        if(dist<bestDist)
                        {
                            bestDist=dist;
                            bestIdx2=i2;
                        }
                    }

                    if(bestDist<=ORBdist)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                        nmatches++;
                        // Step 5 计算匹配点旋转角度差所在的直方图
                        if(mbCheckOrientation)
                        {
                            float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }

                }
            }
        }
```
3. 旋转一致检测，剔除不一致的匹配
```cpp
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                        nmatches--;
                    }
                }
            }
        }
```
## SearchByBow(Tracking)

`SearchByBow`的函数接口为：

```cpp
int ORBmatcher::SearchByBoW(
    KeyFrame* pKF,
    Frame &F,
    vector<MapPoint*> &vpMapPointMatches
)
```

### 1.构造旋转直方图

```cpp
        // 特征点角度旋转差统计用的直方图
        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        // 将0~360的数转换到0~HISTO_LENGTH的系数
        //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码  
        // const float factor = HISTO_LENGTH/360.0f;
        const float factor = 1.0f/HISTO_LENGTH;
```
### 2.对于`pKF`与`F`的`FeatureVector `，对属于**同一节点**的ORB特征进行匹配

为什么用`FeatureVector`能加快搜索？从词汇树中可以看出，`Node`相对于`Word`为更加抽象的簇，一个`Node`下包含了许多的**相似的**`Word`。如果想比较两个东西，那么先用**抽象特征**进行粗筛，然后再**逐步到**具体的特征。两帧图像特征匹配类似，先对比`FeatureVector`中的`Node`，如果`Node`为同一节点，再用节点下`features`进行匹配，这样避免了所有特征点之间的两两匹配
```cpp
        // 取出关键帧的词袋特征向量
        const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;
        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        // 将属于同一节点的ORB特征进行匹配
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
        DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
        DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

        while(KFit != KFend && Fit != Fend)
        {
            // Step 1：分别取出属于同一node的ORB特征点
            if(KFit->first == Fit->first) 
            {
            	...
                KFit++;
                Fit++;
            }
            else if(KFit->first < Fit->first)
            {
                // 对齐
                KFit = vFeatVecKF.lower_bound(Fit->first);
            }
            else
            {
                // 对齐
                Fit = F.mFeatVec.lower_bound(KFit->first);
            }
        }
```
### 3.对同一node，用`KF`中**地图点对应的ORB特征点**与`F`中的**ORB特征点**两两匹配

其条件：

* 不能重复匹配`vpMapPointMatches`：如果`vpMapPointMatches[realIdxF]`非`NULL`，说明已有匹配了，则不能再匹配
* 最佳匹配距离小于阈值`TH_LOW`
* 最佳匹配距离与次佳匹配距离的比值小于阈值`mfNNratio`

```cpp
                // second 是该node内存储的feature index
                const vector<unsigned int> vIndicesKF = KFit->second;
                const vector<unsigned int> vIndicesF = Fit->second;

                // Step 2：遍历KF中属于该node的特征点
                for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
                {
                    // 关键帧该节点中特征点的索引
                    const unsigned int realIdxKF = vIndicesKF[iKF];

                    // 取出KF中该特征对应的地图点
                    MapPoint* pMP = vpMapPointsKF[realIdxKF];

                    if(!pMP)
                        continue;

                    if(pMP->isBad())
                        continue;
                    // 取出关键帧KF中该特征对应的描述子
                    const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF); 

                    int bestDist1=256; // 最好的距离（最小距离）
                    int bestIdxF =-1 ;
                    int bestDist2=256; // 次好距离（倒数第二小距离）

                    int bestDist1R=256;
                    int bestIdxFR =-1 ;
                    int bestDist2R=256;
                    // Step 3：遍历F中属于该node的特征点，寻找最佳匹配点
                    for(size_t iF=0; iF<vIndicesF.size(); iF++)
                    {
                        if(F.Nleft == -1){
                            // 这里的realIdxF是指普通帧该节点中特征点的索引
                            const unsigned int realIdxF = vIndicesF[iF];

                            // 如果地图点存在，说明这个点已经被匹配过了，不再匹配，加快速度
                            if(vpMapPointMatches[realIdxF])
                                continue;
                            // 取出普通帧F中该特征对应的描述子
                            const cv::Mat &dF = F.mDescriptors.row(realIdxF);
                            // 计算描述子的距离
                            const int dist =  DescriptorDistance(dKF,dF);

                            // 遍历，记录最佳距离、最佳距离对应的索引、次佳距离等
                            // 如果 dist < bestDist1 < bestDist2，更新bestDist1 bestDist2
                            if(dist<bestDist1)
                            {
                                bestDist2=bestDist1;
                                bestDist1=dist;
                                bestIdxF=realIdxF;
                            }
                            // 如果bestDist1 < dist < bestDist2，更新bestDist2
                            else if(dist<bestDist2)
                            {
                                bestDist2=dist;
                            }
                        }
                        else{
                            const unsigned int realIdxF = vIndicesF[iF];

                            if(vpMapPointMatches[realIdxF])
                                continue;

                            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                            const int dist =  DescriptorDistance(dKF,dF);

                            if(realIdxF < F.Nleft && dist<bestDist1){
                                bestDist2=bestDist1;
                                bestDist1=dist;
                                bestIdxF=realIdxF;
                            }
                            else if(realIdxF < F.Nleft && dist<bestDist2){
                                bestDist2=dist;
                            }

                            if(realIdxF >= F.Nleft && dist<bestDist1R){
                                bestDist2R=bestDist1R;
                                bestDist1R=dist;
                                bestIdxFR=realIdxF;
                            }
                            else if(realIdxF >= F.Nleft && dist<bestDist2R){
                                bestDist2R=dist;
                            }
                        }

                    }
                    // Step 4：根据阈值 和 角度投票剔除误匹配
                    // Step 4.1：第一关筛选：匹配距离必须小于设定阈值
                    if(bestDist1<=TH_LOW)
                    {
                        // Step 4.2：第二关筛选：最佳匹配比次佳匹配明显要好，那么最佳匹配才真正靠谱
                        if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                        {
                            // Step 4.3：记录成功匹配特征点的对应的地图点(来自关键帧)
                            vpMapPointMatches[bestIdxF]=pMP;

                            // 这里的realIdxKF是当前遍历到的关键帧的特征点id
                            const cv::KeyPoint &kp =
                                    (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                    (realIdxKF >= pKF -> NLeft) ? pKF -> mvKeysRight[realIdxKF - pKF -> NLeft]
                                                                : pKF -> mvKeys[realIdxKF];
                            // Step 4.4：计算匹配点旋转角度差所在的直方图
                            if(mbCheckOrientation)
                            {
                                cv::KeyPoint &Fkp =
                                        (!pKF->mpCamera2 || F.Nleft == -1) ? F.mvKeys[bestIdxF] :
                                        (bestIdxF >= F.Nleft) ? F.mvKeysRight[bestIdxF - F.Nleft]
                                                            : F.mvKeys[bestIdxF];
                                // 所有的特征点的角度变化应该是一致的，通过直方图统计得到最准确的角度变化值
                                float rot = kp.angle-Fkp.angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);// 将rot分配到bin组, 四舍五入, 其实就是离散到对应的直方图组中
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxF);
                            }
                            nmatches++;
                        }

                        if(bestDist1R<=TH_LOW)
                        {
                            if(static_cast<float>(bestDist1R)<mfNNratio*static_cast<float>(bestDist2R) || true)
                            {
                                vpMapPointMatches[bestIdxFR]=pMP;

                                const cv::KeyPoint &kp =
                                        (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                        (realIdxKF >= pKF -> NLeft) ? pKF -> mvKeysRight[realIdxKF - pKF -> NLeft]
                                                                    : pKF -> mvKeys[realIdxKF];

                                if(mbCheckOrientation)
                                {
                                    cv::KeyPoint &Fkp =
                                            (!F.mpCamera2) ? F.mvKeys[bestIdxFR] :
                                            (bestIdxFR >= F.Nleft) ? F.mvKeysRight[bestIdxFR - F.Nleft]
                                                                : F.mvKeys[bestIdxFR];

                                    float rot = kp.angle-Fkp.angle;
                                    if(rot<0.0)
                                        rot+=360.0f;
                                    int bin = round(rot*factor);
                                    if(bin==HISTO_LENGTH)
                                        bin=0;
                                    assert(bin>=0 && bin<HISTO_LENGTH);
                                    rotHist[bin].push_back(bestIdxFR);
                                }
                                nmatches++;
                            }
                        }
                    }

                }
```
### 4.旋转一致检测，剔除不一致的匹配
![在这里插入图片描述](https://img-blog.csdnimg.cn/5cfbe40185ea47d895fd84f63f7abbbe.png)

```cpp
        // Step 5 根据方向剔除误匹配的点
        if(mbCheckOrientation)
        {
            // index
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                // 如果特征点的旋转角度变化量属于这三个组，则保留
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;

                // 剔除掉不在前三的匹配对，因为他们不符合“主流旋转方向”  
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
```
# 优化

## Vertex

### 重投影
#### VertexSE3Expmap
| 优化变量 | 类型      | 维度 |
| :------- | :-------- | :--- |
| $T_{cw}$ | `SE3Quat` | 6    |
* `VertexSE3Expmap`：SE3类型顶点在内部用变换矩阵参数化，在外部用指数映射参数化
* `SE3Quat`用**四元数**表示旋转，在更新时将**6维的前3维**通过李群李代数进行指数映射为旋转矩阵，然后再转换为四元数，内部操作采用四元数
* 更新`oplusImpl`：
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

### IMU

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

### 重力

### 尺度

### Sim3

## Edge
### 重投影
#### EdgeSE3ProjectXYZOnlyPose

`EdgeSE3ProjectXYZOnlyPose`为一元边

| 成员               | 说明                                          | 类型                   |
| ------------------ | --------------------------------------------- | ---------------------- |
| 顶点`_vertices`    | $T_{cw}$                                      | `g2o::VertexSE3Expmap` |
| 观测`_measurement` | $p=\left(u,v\right)$                          | `Eigen::Vector2d`      |
| 残差`_error`       | $err = p-\pi \left(T_{cw} \cdot P_{w}\right)$ | `Eigen::Vector2d`      |

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
### IMU
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


## PoseOptimization

`PoseOptimization`主要的作用是利用**重投影**优化**单帧的位姿**，主要用在`Tracking`的几种跟踪模式`TrackWithMotionModel`、`TrackReferenceKeyFrame`、 `TrackLocalMap`、`Relocalization`中
![](https://img-blog.csdnimg.cn/42ae5eb09cab453abc35493835d222f3.png)

### 输入  
| 优化变量 |              | 观测         |
| :------- | :----------- | :----------- |
| 帧的Pose | 帧的MapPoint | 帧的KeyPoint |
### 初始化  
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
### 设置vertex
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

### 设置edge
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

### 优化策略
* 分4次优化，每次迭代10次
* 每次优化，评估每条重投影边的残差
	* 如果大于阈值，设置为`level = 1`，不再参与优化
	* 如果小于阈值，设置为`level = 0`
* 从第**3**次开始，不再使用鲁棒核函数`e->setRobustKernel(0)`
### 恢复
```cpp
g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));  
g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();  
cv::Mat pose = Converter::toCvMat(SE3quat_recov);  
pFrame->SetPose(pose);
```
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















