1. debug mode:
编译: 
colcon build --packages-select localization_2d --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
运行: 
(1). 使用 gdb 启动节点
gdb --args /home/linjs/catkin_ws/install/localization_2d/lib/localization_2d/localization_2d_node

(2). 在 GDB 提示符 (gdb) 下输入 run 启动程序
(gdb) run

(3). 程序发生段错误崩溃时，GDB 会拦截它。此时输入 bt (backtrace) 查看完整的调用栈
(gdb) bt