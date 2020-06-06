# LIBVISO2

**[LIBVISO2 (Library for Visual Odometry 2)](http://www.cvlibs.net/software/libviso/)** 是AVG(Autonomous Vision Group)小组的开源项目。

它是一个用于计算移动单目/双目相机的6DoF运动的跨平台C++库，libviso2 不依赖其他CV库，独立的实现了视觉里程计。这是一个非常简单的开源的视觉里程计算法，非常简单啊，特别适合初学者作为入门的第一款视觉里程计算法。

本文将详细介绍 libviso2 双目视觉里程计部分的实现原理和运动估计部分代码讲解。

# 一起读代码

打开工程，在/src下面可以看见libviso2的C++源代码，密密麻麻一大堆头文件和源文件。

<img src="img/tree.png" alt="tree" style="zoom:75%;" />

下载一下test用到的数据库，再编译运行一下，可以看到 Current pose 被连续不断的打印到屏幕上。

[pic] //TODO

别慌，咱们一起来看看这个代码的运行流程，先从main() 函数所在的 demo.cpp 开始。

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









# 原理

## 特征匹配部分

### 特征提取

+ [高斯拉普拉斯算子](http://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm)

  Laplace算子作为一种优秀的边缘检测算子，在边缘检测中得到了广泛的应用。该方法通过对图像求图像的二阶倒数的零交叉点来实现边缘的检测，公式表示如下：

  ![](http://latex.codecogs.com/gif.latex?\LARGE%20\bigtriangledown^2f=\frac{\partial^2f}{\partial%20x^2}+\frac{\partial^2f}{\partial%20y^2})

  由于Laplace算子是通过对图像进行微分操作实现边缘检测的，所以对离散点和噪声比较敏感。于是，首先对图像进行高斯卷积滤波进行降噪处理，高斯函数的表达式如下：

  

  ![](http://latex.codecogs.com/gif.latex?\LARGE%20G_\sigma(x,y)=\frac{1}{\sqrt{2\pi\sigma^2}}e^{-\frac{x^2+y^2}{2\sigma^2}})

  再采用Laplace算子进行边缘检测，就可以提高算子对噪声和离散点的鲁棒性，如此，拉普拉斯高斯算子Log（Laplace of Gaussian）就诞生了。

+ Sobel 滤波

libviso2 采用 Blob/Corner Detector 两种特征模板来提取特征。

<img src="../../Desktop/slamjieshao/jj/image-20200606225855187.png" alt="image-20200606225855187" style="zoom: 67%;" />

如图所示，Blob Musk 是一个高斯拉普拉斯算子。

### 环形匹配



### 快速匹配



## 运动估计部分

### 特征 Bucketing



### 3D Points Calcuation



### Projection Model



### Minimize Reprojection Errors



### RANSAC



### Gauss-Newton optimization






# 关键部分代码详细注释和讲解

// TODO