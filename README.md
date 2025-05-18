# 使用

## ROS2 Launch 说明

### 使用复杂版 ros2 launch
说明：需要修改 setup.py 将 demo.launch.py 拷贝到 install 文件夹爱
```bash
# 进入工作目录
source install/setup.bash
ros2 launch test_pkg demo.launch.py
```

### 使用简化版 ros2 launch
```bash
# 进入包中的 launch 文件夹
ros2 launch launch.py 
```

## 项目使用方法

### ROS2 文件使用方法
```bash
./test.sh
```

### ROS2 Rqt Plot 可视化
```bash
rqt -plot

# Plugin -> Visualization -> Plot
```

### ROS2 Debug
1. 安装 ROS 插件 (Microsoft 官方插件)  
2. 点击左侧 Run and Debug 图标 -> Create launch.json file -> ROS -> ROS2: launch -> 选择debug的包 -> 选择启动的 launch 文件