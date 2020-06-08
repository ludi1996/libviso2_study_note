# LIBVISO2 学习笔记

**[LIBVISO2 (Library for Visual Odometry 2)](http://www.cvlibs.net/software/libviso/)** 是AVG(Autonomous Vision Group)小组的开源项目。

它是一个用于计算移动单目/双目相机的6DoF运动的跨平台C++库，libviso2 不依赖其他CV库，独立的实现了视觉里程计。这是一个非常简单的开源的视觉里程计算法，非常简单啊，特别适合初学者作为入门的第一款视觉里程计算法。

本文将详细介绍 libviso2 双目视觉里程计部分的实现原理和运动估计部分代码讲解。

# 代码结构

打开工程，在/src下面可以看见libviso2的C++源代码，密密麻麻一大堆头文件和源文件。

<img src="img/tree.png" alt="tree" style="zoom:75%;" />

下载一下test用到的数据库，再编译运行一下，可以看到 Current pose 被连续不断的打印到屏幕上。看起来很复杂是吗？别慌，让我们一起来看看这个代码的运行流程，先从main() 函数所在的 demo.cpp 开始。

+ demo.cpp

```c++
// demo.cpp
// include <balabala>
// using namespace
int main (int argc, char** argv) {
	init_parameters();	// 40-65行，初始化各种参数
    for (int32_t i=0; i<373; i++) { 	// 68行开始循环
    	read_images();	// 70-96行，读取图片，并且将图片转换数据格式
        // 在这里，viso.process() 是核心，在proscess()函数中完成了所有的主要工作
        // 传入读取到的当前帧的图片，proscess()函数就会计算出前帧图片的位姿变换
        // 并将各种参数更新，保存在VO对象属性的变量中，并返回一个bool类型
        if (viso.process(left_img_data,right_img_data,dims)) {
        	// process()函数处理当前帧图片的位姿变换，处理成功则返回true
            pose = pose * Matrix::inv(viso.getMotion());	// 更新当前的位姿
        	print_information();	// 108-113行，打印信息
        }
        else {
            // process()失败则不更新，跳过这帧处理下一帧
        	cout << " ... failed!" << endl;		
      	}
        release_buffers();	//释放内存
        catch();	// 有错报错
    }
    return 0;	// 结束
}
```

看完了demo.cpp，我们惊奇的发现，我们看懂了99%的代码，但是对视觉里程计一无所知，所有能看懂的都是废话，等于没看。唯独 viso.process() 函数不知道是什么，但是关键的关键都包含在了 viso.process() 函数中。那么就让我们跳转到 viso.process() 函数中看看。

+ viso_stereo.cpp

```c++
// viso_stereo.cpp
bool VisualOdometryStereo::process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace) {
  // 35-50行，也就是从这里开始到return 之前，都是在做图像特征的提取和匹配
  matcher->pushBack(I1,I2,dims,replace);	// 特征提取
  // 两次匹配策略加速匹配
  if (!Tr_valid) {	// 第一次匹配
    matcher->matchFeatures(2);	// 特征匹配
    // Bucketing处理
    matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);
    p_matched = matcher->getMatches();	// 更新匹配点
    updateMotion();
  }
  // 第二次匹配
  if (Tr_valid) matcher->matchFeatures(2,&Tr_delta);
  else          matcher->matchFeatures(2);
  matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);
  p_matched = matcher->getMatches();
  // 得到匹配好的点后，就可以进行运动估计了
  // 运动估计的部分被包含在了updateMotion() 函数中
  return updateMotion();
}
```

process() 函数中分为两部分，特征匹配和运动估计。99%的代码是在做特征匹配，而运动估计部分只在最后一行的 return 中 调用了 updateMotion() 函数。updateMotion() 函数定义在了 VisualOdometryStereo 类的父类 VisualOdometry 中，并且没有在子类中被重写，那让我们跳转到updateMotion() 中。

+ viso.cpp

```c++
// viso.cpp
bool VisualOdometry::updateMotion () {
  // updateMotion () 调用 estimateMotion() 返回估计好的位姿变换 T和r
  vector<double> tr_delta = estimateMotion(p_matched);
  if (tr_delta.size()!=6)	//返回的T和r无效返回false
    return false;

  // 返回的T和r有效则更新位姿变换，再返回true
  Tr_delta = transformationVectorToMatrix(tr_delta);
  Tr_valid = true;
  return true;
}
```

updateMotion() 函数中的主要工作又被封装到了 estimateMotion() 函数中了，estimateMotion()函数才是做了真正的运动估计，返回了估计好的位姿变换T和r。updateMotion() 函数再根据估计好的位姿的有效性（运动估计无效 estimateMotion() 会返回空向量）做一个更新和返回。那再继续去看estimateMotion() 函数吧。

+ viso_stereo.cpp

```c++
vector<double> VisualOdometryStereo::estimateMotion (vector<Matcher::p_match> p_matched) {
  
  bool success = true;  // 声明返回值
  
  min_distance();	// 59-65行，计算RANSAC的最小距离
  
  int32_t N  = p_matched.size();	// 67-70行，得到匹配点的数量，必须大于等于6个

  memory = new <type>[N];	// 72-79行，创建动态变量用于计算

  for (i=0; i<N; i++) {
    project_points(points[i]);	// 81-87行，将前一时刻两图中匹配好的特征点投影成3D点，计算出这些特征点的3D坐标
  }

  before_loop();	// 89-95行，准备开始RANSAC循环，声明临时变量

  for (int32_t k=0;k<ransac_iters;k++) {	// 97-114行，RANSAC迭代进行运动估计
    RANSAC();		// RANSAC算法，后面详细介绍
    while (result==UPDATED) {
      result = updateParameters();	// 107-114行，updateParameters()函数实现了高斯牛顿法迭代优化
    }
    overwrite_best_param();		// 116-123行，RANSAC算法中，将参数更新为内点最多的一组
  }
  
  refinement();		// 126-134行，再优化
  return();			// 136-157行，delete临时变量，返回
```

estimateMotion() 函数中，先将前一时刻的特征点投影到三维空间中。然后开始RANSAC循环，RANSAC循环中嵌套了一个while循环，在while循环中updateParameters()函数实现了高斯牛顿法，通过最小化重投影误差求解R和t。就这样，estimateMotion() 函数使用了RANSAC算法和高斯牛顿法实现了运动估计。

# 实现原理

+ 一般的基于特征点的VO算法分为如下几个步骤：

![](img/image-20200604050006111.png)

## Feature Matching

+ 高斯拉普拉斯算子

  Laplace算子作为一种优秀的边缘检测算子，在边缘检测中得到了广泛的应用。该方法通过对图像求图像的二阶倒数的零交叉点来实现边缘的检测，公式表示如下：

  ![](img/w9.gif)

  由于Laplace算子是通过对图像进行微分操作实现边缘检测的，所以对离散点和噪声比较敏感。于是，首先对图像进行高斯卷积滤波进行降噪处理，高斯函数的表达式如下：

  

  ![](img/w8.gif)

  

  再将Laplace算子应用于高斯卷积滤波后的图像：

  ![](img/w7.gif)

  如此，[拉普拉斯高斯算子LoG](http://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm)（Laplace of Gaussian）就诞生了。这样就可以提高Laplace算子对噪声和离散点的鲁棒性。

   2D高斯拉普拉斯算子可以通过任何一个方形核进行逼近，只要保证该核的所有元素的和或均值为0，如后文的Blob Musk就是一个5x5的核逼近的拉普拉斯高斯算子。

+ Sobel Filter

  [Sobel 滤波](https://homepages.inf.ed.ac.uk/rbf/HIPR2/sobel.htm)是获得数字图像的一阶梯度的方法，使用索贝尔算子把图像中每个像素的上下左右四领域的灰度值加权求和，来运算图像亮度函数的梯度之近似值。

  该算子包含一对儿3x3的矩阵，分别为横向及纵向，如下图所示，将之与图像作平面卷积（将kernels应用于图像），即可分别得出横向及纵向的亮度差分近似值。

  <img src="img/image-20200607010427009.png" alt="image-20200607010427009" style="zoom: 80%;" />

  图像梯度的大小和方向可由横向梯度Gx与纵向梯度Gy结合得到：

  ![](img/w6.gif)

  ![](img/w5.gif)

+ SAD technique

  SAD(Sum of absolute differences)是一种图像匹配算法。其基本思想是比较特征模板差的绝对值之和。此算法常用于图像块匹配，将每个像素对应数值之差的绝对值求和，据此评估两个图像块的相似度。
 ###  Feature Detection

+ Blob/Corner Detector

  <img src="img/image-20200606225855187.png" alt="image-20200606225855187" style="zoom: 50%;" />
  
  libviso2 采用 Blob/Corner Detector 两种特征模板来提取特征。如图所示，Blob Musk 是一个高斯拉普拉斯算子，它和 Corner Musk 分别检测边缘点和角点。采用非极大值抑制和非极小值抑制的方法保留局部极大值和极小值作为特征点。
  
+ Feature Descriptor

  ![img](img/FeatureDescriptor.png)

  采用如上图所示的模板作为特征点的描述子，在11x11的矩形中的16个位置，保留了图像的一阶梯度值（Sobel滤波），并且使用SAD算法进行特征模板匹配。为了加速特征匹配，特选取8 bits 的 Sobel 算子进行 Sobel 滤波，选取稀疏的16个点来代表整个矩形。这是因为在SAD算法中使用 16 bytes 的特征描述子做特征点匹配，计算机只需要调用一条简单的 SSE 指令就可以实现非常高效的进行计算。

### Circle Match

+ Circle Strategy

  ![img](img/CircleMatch.png)

  ​	双目相机的特征匹配要对4幅图像进行匹配，环形匹配的策略是：从当前时刻的左图出发，对于每一个的特征点，先去前一时刻的左图中划一个M×M的窗口，在窗口中寻找最佳匹配点；再到前一时刻的右图中划定一个M×N的窗口，寻找到最佳匹配点；然后去当前时刻的右图中的M×M窗口中进行匹配；最后回到当前时刻的左图中M×N窗口内匹配。



+ Reject Mismatch

  ![img](img/CircleMatchOrNot.png)

  如果环形匹配结束点和起始点是同一个点（允许1 pixel 的误差），那么这组匹配点作为正确匹配的特征点保留。否则认为是匹配失败，将这组点剔除。

### Fast Feature Matching

​	在特征匹配过程中，libviso2使用有效的位姿变换信息来缩小特征匹配的搜索区域，以提高特征匹配速度。快速匹配策略将特征匹配分为两个阶段：

+ 在第一阶段中，算法选取一部分稀疏的特征点进行匹配，根据这一部分点的匹配信息，计算出相机前后时刻大致的位姿变换。

+ 在第二阶段中，算法根据有效的相机位姿变换，预测出当前时刻左图中的特征点在其他3幅图中可能出现的区域，在此区域内进行搜索匹配。

​	先用少数特征点进行全局匹配，再计算出大致的前后时刻相机位姿变换，然后对剩余的所有特征点，利用相机位姿变换信息缩小搜索的区域，这样可大幅提高特征匹配的速度。



## Egomotion Estimation

![img](img/Egomotion.png)

​	Libviso2 通过最小化重投影误差的方法优化求解相机运动。特征匹配完成之后，使用Bucketing策略减少特征点，再进行运动估计。

+ 首先根据前一时刻左右两图匹配好的特征和特征点的视差，将所有特征点投影到三维空间中，计算出所有特征点在相机坐标系下的三维坐标；
+ 相机发生位姿变换后，再将这些三位空间中的特征点重投影到运动后的相机成像平面；
+ 最小化计算得到的重投影特征点和实际特征点之间的误差，使用RANSAC算法去除外点，再用高斯牛顿法优化求解 R 和 t；
+ 最后经过一次再优化得到最终的 R 和 t ，完成相机运动估计。

### Features Bucketing

<img src="img/20191205142754303.png" alt="img"  />

在做运动估计计算之前，首先采用Bucket策略进一步减少或者说均衡特征点。图像被均分为很多长方形区域，每个区域限制特征点个数的最大值。如上图所示，图像被分割成若干个矩形区域，在每个矩形区域中只保留最多n个点。

+ 这样做的好处是
  + 减少特征点的总数量，提高算法效率
  + 使特征点均匀的分布在图像中，避免很多特征点扎堆出现
  + 均衡不同深度的特征点分布，远近特征点均有机会被运动估计算法采用，从而提高了估计精度。

### 3D Points Calcuation

根据已匹配特征点的视差，计算出此点在三维空间内的坐标

+ disparity

  ![](img/w4.gif)

+ Stereo Baseline

  ![](img/w3.gif)

+ 3d points

  ![](img/w2.gif)

### Projection Model

![](img/w1.gif)

​	投影模型描述了将3维空间中的点投影到2维平面图像上的关系。上式表示，在相邻的两时刻相机发生了位姿变换 R 和 t ，前一时刻相机坐标系空间中的点P(Xp,Yp,Zp)，在后一时刻重投影到相机成像平面所在的坐标(Uc,Vc)。投影到左图时 s = 0；投影到右图时 s = stereomodel.baseline。

### Minimize Reprojection Errors

- reprojection errors

  ![e7](img/e7.gif)

  上式为重投影误差，![img](img/p_2i.gif)是当前时刻图像中特征点的坐标；![](img/pi2.gif)是前一时刻的特征点投影到3d中，再重投影到变换后的相机平面的坐标。分别计算左右两图中的重投影误差的平方和即为误差函数。

### RANSAC

​	[RANSAC算法](http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FISHER/RANSAC/)非常适合用来拟合存在许多离群点的数据集。由于匹配算法本身存在的误差，运动物体的干扰，VO算法会采用RANSAC算法去除外点。

<img src="img/image-20200608051237646.png" alt="image-20200608051237646" style="zoom: 40%;" />

RANSAC算法流程很简单，下面介绍一下在libviso2运动估计中使用的RANSAC算法：

1. 从所有匹配好的特征点中，随机挑选3组特征点。
2. 使用这3组匹配点，计算相机的运动 R 和 t （最小化重投影误差，高斯牛顿法求解R、t）。
3. 将计算出的 R 和 t 应用于所有特征点。对于每一个特征点，将前一时刻两幅图中的该点投影到3维空间中，再经过此 R 和 t 变换，判断其空间位置是否与后一时刻两幅图中该点投影到空间中的位置相吻合（欧氏距离小于某阈值）。如果相吻合则称之为内点并保留，否则称之为外点并剔除。统计内点个数。
4. 将步骤1-3重复N次。
5. 迭代N次后，选择包含内点个数最多的一组内点数据，重新估计运动。

之所以随机挑选3组匹配点，是因为刚体运动条件下，求解运动最少需要3组独立的匹配点。

### Gauss-Newton optimization

高斯牛顿法是常用的数值优化算法，该法使用泰勒级数展开式去近似地代替非线性回归模型，用雅可比矩阵经过运算近似代替海塞矩阵，迭代优化解决非线性最小二乘问题。在libviso2中应用高斯牛顿法的最小化问题如下：

+ Least squares problem

  ![E](img/e7.gif)

+ Errors vector

  ![e6](img/e6.gif)

+ Optimization parameters vector

  ![](img/e4.gif)

+ Jocobians Matrix

  ![](img/e3.gif)

+ Parameter iteration

  ![](img/e2.gif)

### Jacobian Matrix Compute

代码实现了雅可比矩阵的计算。

![e1](img/e1.gif)

