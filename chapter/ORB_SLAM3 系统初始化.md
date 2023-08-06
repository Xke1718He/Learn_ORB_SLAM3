
* [ORB SLAM3系统初始化](https://blog.csdn.net/He3he3he/article/details/125583617?spm=1001.2014.3001.5502)
* [ORB SLAM3 构建Frame](https://blog.csdn.net/He3he3he/article/details/130914425?spm=1001.2014.3001.5502)
* [ORB_SLAM3 单目初始化](https://blog.csdn.net/He3he3he/article/details/129918150?spm=1001.2014.3001.5502)
* [ORB_SLAM3 双目匹配](https://blog.csdn.net/He3he3he/article/details/130416596?spm=1001.2014.3001.5502)
* [ORB_SLAM3_IMU预积分理论推导(预积分项)](https://blog.csdn.net/He3he3he/article/details/130047551?spm=1001.2014.3001.5502)
* [ORB_SLAM3_IMU预积分理论推导(噪声分析)](https://blog.csdn.net/He3he3he/article/details/130047460?spm=1001.2014.3001.5502)
* [ORB_SLAM3_IMU预积分理论推导(更新)](https://blog.csdn.net/He3he3he/article/details/130047581?spm=1001.2014.3001.5502)
* [ORB_SLAM3_IMU预积分理论推导(残差)](https://blog.csdn.net/He3he3he/article/details/130054238?spm=1001.2014.3001.5502)
* [ORB_SLAM3_优化方法 Pose优化](https://blog.csdn.net/He3he3he/article/details/130182435?spm=1001.2014.3001.5502)
* [ORB_SLAM3 闭环检测](https://blog.csdn.net/He3he3he/article/details/131158773?spm=1001.2014.3001.5502)

ORB SLAM3的初始化主要是为了：
* 从配置文件中读取需要的**参数**
* 创建**跟踪线程，局部建图线程，闭环线程**三大主线程，以及可视化线程
* 创建后面需要的对象：`ORB词袋、关键帧数据库、多地图`等

其步骤如下：
* 步骤：
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
### A.相机模型
从配置文件中读取`相机参数`，创建相机模型(`Pinhole/KannalaBrandt8`)
* `Pinhole/KannalaBrandt8`： 继承自`GeometricCamera`，输入：相机参数（`fx,fy,cx,cy, k1, k2, p1, p2, k3`/`fx,fy,cx,cy,k1,k2,k3,k4`）
* 添加相机模型到多地图系统中(`mpAtlas->AddCamera(mpCamera)`)

对于针孔相机模型`Pinhole`可以参考文章：[【二】[详细]针孔相机模型、相机镜头畸变模型、相机标定与OpenCV实现](https://blog.csdn.net/He3he3he/article/details/98769173)

```
OpenCV 中的 `FileStorage` 类能够读写`.xml`和`.yaml`文件，以 `FileNode` 为单位存储数据，其有以下操作：
* 写入:`FileStorage::WRITE`
* 追加:`FileStorage::APPEND`
* 读取:`FileStorage::WRITE`
```

### B.ORB特征提取器

从配置文件中读取`ORB参数`，创建ORB特征提取器(`ORBextractor`)，其主要参数：
* **nFeatures**: 特征点的数量
* **Scalefactor**：特征金字塔的尺度因子
* **nLevels**: 特征金字塔的层数
* **iniThFAST**: 初始Fast阈值
* **minThFAST**: 最小Fast阈值

`ORBextractor`的构造函数如下，需要注意的是单目情况下，特征点的数目增加为5倍
```cpp
mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
//相对于普通情况，单目初始化时提取的特征点数量更多
mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
```
构造`ORBextractor`特征提取器主要分为如下几个步骤：
1. 设置图像金字塔的`尺度因子`、`逆尺度因子`、`方差`
```C++
mvScaleFactor.resize(nlevels);
mvInvScaleFactor.resize(nlevels);

mvLevelSigma2.resize(nlevels);
mvInvLevelSigma2.resize(nlevels);
```
$$
\begin{array}{l}
mvScalefactor_{i} = mvScalefactor_{i-1} \cdot scalefactor \\
mvInvScalefactor_{i} = \frac{1}{mvScalefactor_{i}} \\
mvLevelSigma2_{i} = mvScalefactor_{i}^{2} \\
mvInvLevelSigma2_{i} = \frac{1}{mvLevelSigma2_{i}} 
\end{array}
$$
**图像金字塔**如下：
![](https://img-blog.csdnimg.cn/img_convert/740b8d6c077f469ac02e1c9f0ca4265a.png)
2. `预分配`每层金字塔的特征点数量
* 分配的方式：先分配前$n-1$层，再将剩余的特征点$N-sum(0,n-1)$分配给第n层。

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
在ORB-SLAM3 的代码里，`不是按照面积均摊的`，而是`按照面积的开方`来均摊特征点的，也就是将上述公式中的$s^2$换成$s$ 即可。

3. 初始化pattern
pattern为$32\cdot8\cdot4 = 1024$，其中pattern是ORB预设好的。
参考文献：《BRIEF: Binary Robust Independent Elementary Features 》


![](https://img-blog.csdnimg.cn/img_convert/b62ce577715e34aca0ee591b3c67fd33.png)


4. 预先计算灰度质心法每行对应的终点
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






