### 哨兵 Minpc 上启动odin1驱动 （ 默认关闭 rviz 用 foxglove）
---
- 拉取 sentry-minpc 分支，编译
- 同步后进入 minpc
```bash
# 1
source /rmcs_install/setup.zsh
ros2 launch odin_ros_driver odin1_ros2.launch.py
```
---
- 如果 `host_sdk_sample` 相关报错，执行
```bash
# 2
pkill -f host_sdk_sample
```
- 再重新进行 1 操作
- 还不行就下电重启，再进行重新一遍所有操作（甚至可以重新编译）
---
- 没报错就打开 `foxglove_bridge`
```bash
# 3
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
- 连接`ws://192.168.3.27:8765`，即可查看 odin1 的 topic
---
- yuyuyu 已在 5.9 日 17：50 通过