# 安装UE4和编译Carla
按照[官方流程](https://carla.readthedocs.io/en/latest/how_to_build_on_linux/)安装UE4和编译carla工程，
   注意：
   + 在安装和编译过程中需要终端有http代理，能访问部分外网(google，unrealengine) 
   + 安装过程会下载数个超大文件，建议仔细查找*Setup.sh*等文件中要下载的文件以及运行时出现的下载错误，然后手动下载放到指定目录
# 运行Carla
  
  + 启动UE4中的carla工程   
      - 进入工程目录 *cd /home/stiperception/disk1/carla-source/Unreal/CarlaUE4/*

      - 启动 carla server */home/stiperception/Desktop/CARLA/UnrealEngine-4.18/Engine/Binaries/Linux/UE4Editor "$PWD/CarlaUE4.uproject"*
      - 在UE4 editor中 点击 **Play** 按钮, 则仿真server启动  

  + 启动Python client
      - client和server通过TCP协议通信，默认走127.0.0.1:2000端口，注意保证端口不被占用。
      - 进入 client目录，*cd /home/stiperception/disk1/carla-source/PythonClient*
      - 运行 lidar_simulator.py 即可进入雷达仿真模式(仿真详细设置已于lidar_simulator.py中说明，恕不赘述)
  
  + RViz可视化&数据获取
      - 在lidar_simulator中已设置相应的节点用来发布ROS sensormsg/PointCloud类型的消息，不同的lidar分别有不同的pubulisher，Rviz中订阅相应的话题即可。(设置多个lidar时，显示过程会产生延迟)
      - 当要导出数据时，可以直接在 lidar_simulator.py 中把点云数据保存成numpy格式的文件，或者用ROSbag录制相应的topic.
# 自定义lidar特性
+ 自定义雷达的特性时，不需要修改UE4引擎，只需要修改Carla工程，具体需要修改源码 *$PROJECT/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/Lidar.cpp*以及相应等文件。
+ 源码中部分类&函数(大多数以F开头)是属于UE4引擎内置类或者函数，可在[官网search](https://www.unrealengine.com/en-US/bing-search?keyword=&offset=0&filter=All)中查找相应的说明、API或开发者论坛。
+ 修改完成后需要重新编译Carla工程(不需要编译整个工程,即不需要运行 Setup.sh),运行根目录下的 Rebuild.sh,编译成功后会自动调用UE4打开Carla工程，之后的步骤可见 “**运行Carla**”
+ 若想自定义python与carla server的接口，可使用lidar_simulator.py 中预设调试变量
lidar.DebugFlag，若要继续增加功能接口，则需要修改carla server中的*LidarDescription.cpp*及相应.h文件。修改完成后要在pthon cilent中修改sensor类，添加新接口($Project/pythonclient/carla/sensor.py)

# LidarSimulator机制
+   GetWorld()->LineTraceSingleByChannel()函数检查某一线雷达以某个角度射出是否会有障碍物

+ 这两个for循环完成了整帧点云的生成

`for(auto Channel = 0u; Channel < ChannelCount; ++Channel)

  {

    for(auto i = 0u; i < PointsToScanWithOneLaser; ++i)

    {
      ;
    }

  }`

 # 建议
 + 看一遍Carla官网的文档，主要看前半部分
 + 仔细分析Lidar.cpp，也就是仿真器如何生成点云

 RSlidar32B  
 
 ![Editor](https://github.com/Super-Tree/LidarSimulator/blob/master/pics/UE4_EDITOR.png)
 ![p3](https://github.com/Super-Tree/LidarSimulator/blob/master/pics/P3.png)
 ![mems](https://github.com/Super-Tree/LidarSimulator/blob/master/pics/mems.png)
 ![rs32b](https://github.com/Super-Tree/LidarSimulator/blob/master/pics/RS32b.png)
 ![Runing](https://github.com/Super-Tree/LidarSimulator/blob/master/pics/runing_time.png)


  
