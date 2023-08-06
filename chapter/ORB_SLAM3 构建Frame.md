## 1.构造Frame
为了构建一帧**Frame**，主要的步骤如下：
1. 提取ORB特征点(`ExtractORB`)
2. 对提取的特征点进行矫正(`cv::undistortPoints`)
3. 计算去畸变后的图像边界(`ComputeImageBounds`)
4. 将特征点分配到网格中(`AssignFeaturesToGrid`)
### A.提取ORB特征点
首先需要对当前帧图像进行特征点的提取：`计算图像金字塔`，`提取Fast角点`，`四叉树均匀化`，`计算特征点的方向`，`计算特征点的描述子`
#### 1.计算图像金字塔
![在这里插入图片描述](https://img-blog.csdnimg.cn/4ec888989ca544f4afab9c1631219a43.png)
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

        // Compute the resized image
		//计算第0层以上resize后的图像
        if( level != 0 )
        {
			//将上一层金字塔图像根据设定sz缩放到当前层级
            resize(mvImagePyramid[level-1],	//输入图像
				   mvImagePyramid[level], 	//输出图像
				   sz, 						//输出图像的尺寸
				   0, 						//水平方向上的缩放系数，留0表示自动计算
				   0,  						//垂直方向上的缩放系数，留0表示自动计算
				   cv::INTER_LINEAR);		//图像缩放的差值算法类型，这里的是线性插值算法

			//把源图像拷贝到目的图像的中央，四面填充指定的像素。图片如果已经拷贝到中间，只填充边界
			//TODO 貌似这样做是因为在计算描述子前，进行高斯滤波的时候，图像边界会导致一些问题，说不明白
			//EDGE_THRESHOLD指的这个边界的宽度，由于这个边界之外的像素不是原图像素而是算法生成出来的，所以不能够在EDGE_THRESHOLD之外提取特征点			
            copyMakeBorder(mvImagePyramid[level], 					//源图像
						   temp, 									//目标图像（此时其实就已经有大了一圈的尺寸了）
						   EDGE_THRESHOLD, EDGE_THRESHOLD, 			//top & bottom 需要扩展的border大小
						   EDGE_THRESHOLD, EDGE_THRESHOLD,			//left & right 需要扩展的border大小
                           BORDER_REFLECT_101+BORDER_ISOLATED);     //扩充方式，opencv给出的解释：
			
			/*Various border types, image boundaries are denoted with '|'
			* BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
			* BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
			* BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
			* BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
			* BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
			*/
			
			//BORDER_ISOLATED	表示对整个图像进行操作
            // https://docs.opencv.org/3.4.4/d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36

        }
        else
        {
			//对于底层图像，直接就扩充边界了
            //?temp 是在循环内部新定义的，在该函数里又作为输出，并没有使用啊！
            copyMakeBorder(image,			//这里是原图像
						   temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
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
#### 2.提取Fast角点
对于金字塔的每一层，将其网格化，每个格子大小为$w = 35$：
* 左上角(**红色**)：$\left(minBorderX, minBorderY\right)$
* 右下角：$\left(maxBorderY, maxBorderY\right)$
* 网格的行数：$rows=\frac{\left(maxBorderY-minBorderY\right)}{w}$
* 网格的列数：$cols=\frac{\left(maxBorderX-minBorderX\right)}{w}$
这时候可以在每个格子中提取Fast角点， 其中格子的范围为：
* $iniX = minBorderX + j \cdot wCell$
* $iniY = minBorderY + i \cdot hCell$
* $maxX = iniX + wCell + 6$
* $maxY = iniY + hCell + 6$
**FAST角点**在`(iniX, iniY, maxX, maxY)`范围内提取，这里使用**高低阈值**
![在这里插入图片描述](https://img-blog.csdnimg.cn/1094569515d9441fa15c6f7e2a9e8a66.png)

```cpp
FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),	//待检测的图像，这里就是当前遍历到的图像块
	vKeysCell,			//存储角点位置的容器
	iniThFAST,			//检测阈值
	true);				//使能非极大值抑制
if(vKeysCell.empty())
{
	//那么就使用更低的阈值来进行重新检测
	FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),	//待检测的图像
		 vKeysCell,		//存储角点位置的容器
		 minThFAST,		//更低的检测阈值
		 true);			//使能非极大值抑制
}
```
提取到的角点的原点位于`紫色`，需要将其变换到`红色`
$$
\begin{array}{c}
x = x + j \cdot wCell \\
y = y + i \cdot hCell
\end{array}
$$
#### 2.四叉树均匀化
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
#### 3.计算特征点方向
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
	//图像的矩，前者是按照图像块的y坐标加权，后者是按照图像块的x坐标加权
    int m_01 = 0, m_10 = 0;

	//获得这个特征点所在的图像块的中心点坐标灰度值的指针center
    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
	//这条v=0中心线的计算需要特殊对待
    //由于是中心行+若干行对，所以PATCH_SIZE应该是个奇数
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
		//注意这里的center下标u可以是负的！中心水平线上的像素按x坐标（也就是u坐标）加权
        m_10 += u * center[u];

    // Go line by line in the circular patch  
	//这里的step1表示这个图像一行包含的字节总数。参考[https://blog.csdn.net/qianqing13579/article/details/45318279]
    int step = (int)image.step1();
	//注意这里是以v=0中心线为对称轴，然后对称地每成对的两行之间进行遍历，这样处理加快了计算速度
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
		//本来m_01应该是一列一列地计算的，但是由于对称以及坐标x,y正负的原因，可以一次计算两行
        int v_sum = 0;
		// 获取某行像素横坐标的最大范围，注意这里的图像块是圆形的！
        int d = u_max[v];
		//在坐标范围内挨个像素遍历，实际是一次遍历2个
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

#### 4.计算特征点的描述子
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
### B.对提取的特征点进行矫正
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
### C.将特征点分配到网格中
将特征点分配到网格中主要的作用：加速特征点的搜索
* `FRAME_GRID_ROWS`：48
* `FRAME_GRID_COLS`：64
$$
\begin{align}
posX & = \left (x-mnMinX\right )\cdot \frac{ {\small FRAME\_GRID\_COLS}  }{mnMaxX-mnMinX} \\
posY & = \left (y-mnMinY\right )\cdot \frac{ {\small FRAME\_GRID\_ROWS}  }{mnMaxY-mnMinY} 
\end{align}
$$


