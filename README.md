## 测试kalman
#### testKalman 原始的kalman滤波测试
```
cd build
rm -rf *
cmake .. && make -j4
./testKalman
```

#### testKalman2 模块化的kalman滤波测试
因为kalman参数和设置比较复杂，来了项目不能什么都不修改，立即就使用，所以这里做一些解释，和使用时需要根据实际项目需要而设置的东西，设置好后才能正常运行，这里一项一项解释，新项目想要使用kalman，需要修改的项，都加了[改]字眼，除了最后一步几乎每一步都需要修改  
 * 第1步初始化	
	+ stateNum: 状态值数目，对于轨迹预测，一般是4个，分别是x,y方向的坐标，x,y方向的速度[改]
	+ measureNum:传感其测量的值数目，对于轨迹预测来说一般是2个，即x,y方向的坐标[改]
	+ controlNum:控制变量的数目，理论上称作外部影响，对于预测轨迹来说就是x,y方向的加速度，理论上是两个值，但现在因为矩阵的乘法的需要，需要写成4个值，有两个值是重复的[改]
	+ initDeltaTime:前一个状态到当前状态的时间间隔，用来生成转移矩阵和和控制矩阵，这里有个问题：是不是需要更新，因为前一个状态到当前状态的时间间隔有可能会变，自己没有搞清楚，觉得应该要更新[改]
	+ 在初始化内部的实现positionKalmanFilter函数里，有时需要修改一下测量矩阵（一般都是单位阵，是预测值到测量值的映射矩阵），系统噪声的强度Q,测量噪声的强度R[改]
 * 第2步更新前后帧的时间间隔，进而更新转换矩阵和控制矩阵[改]  
 * 第3步需要实时外部环境影响，也就是外力，这里指的是加速度，加速度是有两个方向ax,ay，但写成矩阵需要4x1的矩阵，有两个重复[改]  
 * 第4步需要实时更新传感器的测量量，对于轨迹预测来说就是gps预测得的位置x,y[改]  
 * 第5步直接照抄即可，返回值就是当前帧预测的最终状态x,y方向的位置和x,y方向的速度共四个变量，但我们一般只关注前两个即位置信息，直接把前两个取出来使用即可
```
cd build
rm -rf *
cmake .. && make -j4
./testKalman2
```

#### testKalman3 更紧凑的模块化的kalman滤波测试
模块化的测试，功能和testKalman2一样
因为kalman参数和设置比较复杂，来了项目不能什么都不修改，立即就使用，所以这里做一些解释，和使用时需要根据实际项目需要而设置的东西，设置好后才能正常运行，这里一项一项解释，新项目想要使用kalman，需要修改的项，都加了[改]字眼，除了最后一步几乎每一步都需要修改  
 * 第1步初始化	
	+ stateNum: 状态值数目，对于轨迹预测，一般是4个，分别是x,y方向的坐标，x,y方向的速度[改]
	+ measureNum:传感其测量的值数目，对于轨迹预测来说一般是2个，即x,y方向的坐标[改]
	+ controlNum:控制变量的数目，理论上称作外部影响，对于预测轨迹来说就是x,y方向的加速度，理论上是两个值，但现在因为矩阵的乘法的需要，需要写成4个值，有两个值是重复的[改]
	+ initDeltaTime:前一个状态到当前状态的时间间隔，用来生成转移矩阵和和控制矩阵，这里有个问题：是不是需要更新，因为前一个状态到当前状态的时间间隔有可能会变，自己没有搞清楚，觉得应该要更新[改]
	+ 在初始化内部的实现positionKalmanFilter函数里，有时需要修改一下测量矩阵（一般都是单位阵，是预测值到测量值的映射矩阵），系统噪声的强度Q,测量噪声的强度R[改]
 * 第2步直接照抄即可，返回值就是当前帧预测的最终状态x,y方向的位置和x,y方向的速度共四个变量，但我们一般只关注前两个即位置信息，直接把前两个取出来使用即可，需要传入的参数是前后帧的时间间隔，更新转换矩阵和控制矩阵，外部影响(加速度ax,ay),传感器测量值(位置，鼠标位置)
```
cd build
rm -rf *
cmake .. && make -j4
./testKalman3
```

## TODO next
+ NOTHING
## Coding Reference

