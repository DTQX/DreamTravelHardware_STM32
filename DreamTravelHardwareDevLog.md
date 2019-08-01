
##2019.7.31
    1.以后可以用一个圆圈来制作稳定强磁场，避免罗盘受其他磁场的影响

    2.代办：
        1.移植Risk的mpu9250AHRS代码，然后测试精准度
        2.移植Madgwick算法到MPU6050+HMC5883l上，测试精准度（使用dmp的数据、不使用dmp）
        3.测试ICM-20948传感器（使用自带的算法）
        4.从MPU9250+自带算法，MPU6050+HMC5883l+自带算法，MPU9250+Madgwick算法，MPU6050+HMC5883l+Madgwick算法，ICM-20948+自带算法，选出一个解决方案（如果没有合适的，再试试MPU6050+HMC5883l+kalman算法,MPU9250+kalman算法）

    3.标准：
    TODO
      身长2m，圆周长12.5m，允许整体最大偏移10cm，精度0.1/12 = 0.008，角度 0.008*360 = 3度

##2019.7.30
    1.一个发现，今早在跑官方代码看罗盘数据时，发现，绕Z旋转一圈的罗盘数据：0~50。这解释了使用madgwick时，绕z旋转90度，实际只偏移了20度。（50/360约等于20/90）


    2.代办：
        1.自带算法是否满足精度需求？抖动绕z误差15度，复位绕z误差5度。
        2.尝试使用3方罗盘，比较ak89xx、L883（While trying the sensors as a compass, we realised that the 9150 was very noisy and varied between -2 and 2 degrees about a mean value while the hmc was more stable and varied between -0.5 to 0.5 from its mean value. ）

    mpu选择
        1.
        I do like the MPU-6050. Why ? Because it has Accel/Gyro arriving at pretty much the same rate, which can be up to 1000Hz.

        I do NOT like the MPU-9150 to much. The embedded AK8975 is 2.5x less accurate/precises than a HMC5883L/HMC5983 (which can give you 0.5 degrees). For dead reckoning that may be an issue. For me it is as while the rover is stationary the heading will be derived from the magnetometer.

        I do NOT like the MPU-9250 at all. The AK8963 is somewhat better, but still sucks in terms of resolution. I believe 1.5 degrees should be possible with it, but it's rather noisy ... The Accel/Gyro look good on paper, except that they now take insanely long for filtering. Hence you needed to do that on the host again ... So no advantage over the MPU-6050/MPU-9150 combo.

        I do NOT like the ST based products up to the latest generation (the ones they announced, but not ship yet). With a LGD20H/LSM303D (like the Pololu IMUs) or the LSM9DS0 Accel and Gyro have different rates, which makes the effective rate you can get into an EKF or DCM really only 400Hz (2 times oversampling). Even worse, ST does not specify the delay their lowpass/highpass filters cause. That effect of the delay was something observable for me with the MPU-6050 if cranked up all the way into the 10ms range.

        Personally I am back to a GY-86/GY-87 setup (MPU-6050+HMC5883L, BMP180 or MS5611), or simply using a RY835AI GPS/IMU combo. Latter one includes a NEO-M8N GPS/GLONASS setup with a 35mm passive patch antenna, MPU-6050+HMC5983 and a BMP180, all for $50.

        The newer IMUs (MPU-9250, LSM303C, LSM9D1 and such) kind of suck on the magnetometer side as they are build for being put into cell-phones where there are possible huge magnetic distortions. So they have a bigger range there, but less precision. Also they tend to have a bigger focus on size and power consumption, with the end result that built in filtering is slower. Thus the older generation parts might be actually more accurate/precise overall.
    HMC5883L：
        1.12-Bit ADC Coupled with Low Noise AMR Sensors Achieves 2 milli-gauss Field Resolution in ±8 Gauss Fields.
    AK8975A :
        The resolution of the magnetometer is +/- 3 milliGauss with a full-scale range of +/- 12 Gauss. 
    LSM9DS0:
        The magnetometer in the LSM9DS0 offers several output data rates from 3 to 100 Hz and excellent resolution at +/-80 microGauss with a full-scale range up to +/-12 Gauss.
    AK8963:
        allows continuous data output at either 8 or 100 Hz rates and +/-1.5 milliGauss resolution with full-scale range of +/- 48 Gauss, 
    Kris测试：
        mpu9250的效果要比其他的传感器（LSM9DS0）好

##2019.7.29
    1.测试
        1.晃动测试，自带算法很快稳定，Madgwick稳定的慢（考虑是不是delta设置有问题）
        2.Madgwick绕Z轴有问题，mpu转90度，Madgwick得到的只有20度左右
        3.将自带算法得到的数据给Madgwick，效果依然不好
    2.官方自带测试（默认条件，使用dmp ， 不使用timestamp， accelerate 2g， gyro 2000 dps， 使用compass）
        1.compass accuracy = 3 ，晃动测试，绕Z偏移10度左右； accuracy = 0 ，晃动测试，绕Z偏移10度左右。结论：accuracy作用不大
        2.use timestamp 晃动测试，绕Z偏移10度左右; not use 绕Z偏移15度左右。 结论：最好使用timestamp
        3.不使用dmp 晃动测试，绕Z偏移40度以上，且恢复需要一定时间。结论：要使用dmp
        4.accelerate 4g， 绕x，绕y变得不稳定些，绕z和之前一样
        5.gyro scale 暂无法测，预测通过减小scale，可以降低敏感度，从而减小误差
        6.取样率为100及以上时，效果并没有明显提升，且容易卡死
        结论：自带的测试绕Z的误差还是比较大，看看有没有其他替代算法
    3.官方算法，
        1.mpu不动，复位测试
          PlayerBonePoses: X=9.708 Y=-2.591 Z=100.639,
          PlayerBonePoses: X=12.565 Y=-2.380 Z=6.569,
          PlayerBonePoses: X=12.654 Y=-2.327 Z=5.480,
          PlayerBonePoses: X=10.791 Y=-2.593 Z=8.343,
          PlayerBonePoses: X=12.193 Y=-2.454 Z=5.756,
          PlayerBonePoses: X=11.508 Y=-2.378 Z=5.232,
          结论：有几度的偏差
        2.mpu运动，复位
          PlayerBonePoses: X=0.094 Y=0.954 Z=16.275
          PlayerBonePoses: X=0.041 Y=0.961 Z=21.608
          PlayerBonePoses: X=0.025 Y=0.852 Z=16.514
          结论：有几度的偏差，整体效果不错

    3.新方向：
        1.测试kalman滤波
        2.将gyro scale降低为500，看看能不能减小误差

##2019.7.28
    1.网上找的其他算法没移植成功，今天测了mpu9轴自带的9轴融合算法，感觉效果还不错。
    2.其他测试（待测）
        1.mpu9250的i2c上拉电阻换成2k，然后看线长是否满足需求，能否挂载两个。（stm32的i2c速度比较快，导致10k上拉电阻只能接小段距离）
        2.测试TCA9548A，后将程序移植成支持多个mpu。可以使用mpu的i2c master模式，可以直接从mpu中读取罗盘数据，两个mpu是相互隔离的（待测）。
        3.测试mpu官方融合算法的精度是否满足需求，如果不满足，考虑使用3方算法，或者放弃。
    3.准确性测试：
        1.进行剧烈晃动，再放回原位，查看mpu姿态
        2.多次复位，npu保存不动，查看mpu姿态
        3. 调整参数，查看1,2测试

##2019.7.24
    1.官方例程默认使用的是uart2，不是文档上写的1，但pin对上了

## 2019.7.24
    1.mpu6轴传感器的准确度达不到要求，转用mpu9轴传感器（mpu9150）

    2.因mpu9轴融合需要算力，且考虑到以后用usb进行传输，且stm32开发板非常便宜，容易获取 开始转换到stm32开发板，采用stm32f407芯片

    3.最近学习了一些姿态融合算法，对inverse自带的9轴算法、kalman、Madgwick算法进行比较，选择一个最优的。（考虑磁干扰）

    4.目前没有找到合适的stm32 software i2c，所以考虑使用TCA9548A，甚至考虑TCA9548A嵌套使用TCA9548A（一个TCA9548A挂载8个TCA9548A，于是总共可以挂载64的mpu！）

    5.迁移到新工程

## 2019.7.18
    1.mpu bias保存/获取完成，且测试完成，下一步挑选稳定的mpu进行同步测试

## 2019.7.16
    1.目前使用b方案，对dmp sample rate进行了调整，使得每次基本读取一次dmp包，性能得到很大提升；以后再考虑升级主控板

    2.对设置bias的一些思考:
    (1)将bias保存在哪？性能，防盗版，方便性。
        a.将bias放在主控板上，ue4从主控板读取bias，优点：可以利用mpu自带的bias功能，直接得到纠正过的数据，但是还未经考证。缺点：增加了主控板程序难度，不具有硬件防盗版能力
        b.将bias放在服务器上，ue4本地也存一份，优先获取本地，本地没有再去服务器获取。优点：主控板程序会更简单，防盗版能力强。缺点：第一次使用需联网，换主机时需要联网更新。

        目前考虑使用b，arduino due芯片不自带eeprom

## 2019.7.12
    1.多i2c bus可行，读取一个mpu平均时间2200us，速度稍慢，考虑一下方案：a. 每两个使用一个微CPU，使用can传给主控cpu，can的速率为500kbps。b. 使用一个性能更强的mpu。比较价格，复杂度。
      优先考虑b方案，另外再优化i2c代码

    华强报价：
    ARDUINO NANO ： ATmega328 4.2元  4*4mm  16mhz
    arduino zero ：ATSAMD21G18 11元  48 MHz
    arduino mega2560 ： ATMEGA2560  21元  16mhz
    ARDUINO DUE ： AT91SAM3X8E  30元  84mhz

## 2019.7.1
    1.已解决i2c挂多个设备的问题，原硬件i2c上有一个10k的上拉电阻
    2.但是mpu通过AD0引脚动态改变地址会出现问题
    3.打算通过softI2C来解决这个问题，搭建20个i2c总线，每个总线上挂两个mpu

## 2019.6.5
    1.i2c不能挂太多设备，正考虑减小i2c的上啦电阻
    2.新代码已经和ue4调通
    3.研究转变代码为什么可行

## 2019.5.31
    1.对初步的设备进行了测试，i2c不通
    考虑原因：
    1. i2c总线过长
    2. 剥皮接线方式接线不稳定

    计划：
    1. 测试2m的i2c稳定性，如果不稳定，考虑使用两个主控，其中一个主控当i2c中转
    2. 使用穿洞方式接线

## 2019.5.29
    1.各方案已初步确定
    2.正在制作身体部分

## 2019.5.28
    可行性测试标准
    1. 身体部分能实现非物理性的驱动，观测 稳定性、准确性、同步、延迟、舒适性等问题
    2. 手套部分主要观测稳定性、舒适性

    代办
    1.身体线路方案、手套线路方案确定
    2.制作身体部分进行测试。

## 2019.5.27
    计划
    1. 完成同时11个mpu代码，将quat长度缩短一半（32改成16）
    2. 希望做好的设备能用，实在不能用，就戴在手上，看下手指弯曲时，设备的表现（即主要看穿戴表现）
    3. 身体部分仍然采用硬板+连接线，线用28awg（或者30,32）
    4. 别人帮设计的mpu封装，可能有问题，昨天测试，许多mpu的quat一直在变大，所以测下mpu封装，如果封装没问题，那么就是贴片有问题，选出一个最好的。

## 2019.5.26
    初步找到原因
    1.dmp的fifo导致数据混乱
    2.i2c通信问题，官方代码对mpu6050的i2c设置有问题，参考master分支进行修改

    **原因：在启用dmp的函数中 mpu_set_dmp_state(1)时调用了mpu_set_bypass(0)，导致mpu开启了master mode，引起i2c冲突**

## 2019.5.23
    现有问题：i2c读取的dmp经常飘动，不准确
    可能原因：
    1.dmp的fifo溢出（一个现象，在reset fifo后，数据恢复正常），测试方法：将.ino里读取数据之后的延迟去掉，或者监控一次读取fifo的时间
    2.i2c通信问题（新移植的代码，在mpu_init的时候会出i2c错误）,测试方法：使用最原先的代码（master），长时间看看i2c是否出问题。
    3.一个现象：master分支打开串口监控时，没有乱码出现，而自己移植的代码有
    4.Wire库的版本：我台式机上的Wire.cpp与台式机上的居然不一样！测试方法：看看master上用的是哪个版本，与我的进行对比

## 2019.5.20
    移植初步完成！
    部分问题：在初始化设备的时候，要在selectMPU(mpuPins[i]);后加上一定的delay(); 因为digitalWrite(mpuPin, LOW);有一定的延时


身体部分，安装线路

一、电线
1. 线材 28awg
4元/10m，
15*2 * 2 +  4*8*1.5 = 108m

总计 40元，未计入手工费


2. 端子

5p * 50 ： 20元

15 * 2  = 30套 ，

总计：20元， 未计入手工费



二、定制排线

------------------

## 线路方案制定依据

美观、 拆装、 舒适、 稳定、 模块化、 成本

### 身体：
方案一： 普通导线连接（双插座、或者弯电线单插座）               成本：35元，拆装：差，稳定：优，舒适：一般，模块化：一般，成本：不可估
方案二： 普通fpc排线+mpu（单插座）                            成本：35元， 拆装：优，稳定：优，舒适：一般，模块化：优，  成本：低
方案三： 定制fpc+mpu（单插座）                                成本：200以上
方案四： 普通fpc+定制mpu（双插座）                             缺点：排线麻烦，需要不同的连接线，切mpu板会大，不适合手

### 手：
方案一： 整块fpc定制板+mpu在fpc上
方案二： 整块fpc定制板+硬板mpu+单插座
方案三： 普通fpc排线+定制板转接


### 注意点：
1. 因为整体mpu过多，所以mpu一定要模块化

### 总结：
身体：采用方案二，如果fpc排线贵，那么考虑加入转接板；
手： 采用方案三，手掌处加一个转接板，1入，11出（或者10出）；如果mpu实在太大，再另外想办法


## 固定方案

方案一：单层，软布（已军绿大衣为参考），无壳，芯片朝里，       缺点，容易损坏mpu？撞击的时候容易损坏？
方案二：多层。。。困难点：不好扣合
方案三：有壳，单层，                                        缺点，体积大，布的宽度只能和壳一样，对线没有包裹，硬，配带不舒服，撞击的时候会给用户带来较大的痛感

### 总结：
采用方案一，是否容易损坏待测试，如果容易损坏，则加入保护机制