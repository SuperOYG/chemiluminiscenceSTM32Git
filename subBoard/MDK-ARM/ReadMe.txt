程序说明：
这个例程是单人份化学发光的子程序，用于单组的电机和温度控制。
下载程序时注意:
如果给最左侧CPU下载程序：将CAN ID号：设置为0x01;
如果给中间上方CPU下载程序：将CAN ID号：设置为0x02;
如果给最右侧CPU下载程序：将CAN ID号：设置为0x03;

这里的CANID 即作为CAN数据帧的帧ID，同时也作为子板的板号。 
例如后续不同组的同一个功能，可能会有不同的处理方式，例如温度参数的拟合问题，也通过这个帧ID 去做选择判断。

代码修改备注：
2023-05-17： 
在子板程序中，不在使用宏定义去定义，CAN_ID:
