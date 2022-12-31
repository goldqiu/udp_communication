# udp_communication
udp_communication

### 运行

发送端：

```
roslaunch udp_communication message_send.launch
```

接收端：

```
roslaunch udp_communication message_server.launch
```

### 效果

接收端终端得到hello_ausim的文字输出，并获得发送端的ip地址。

### 配置

#### message_send_node.cpp

```cpp
 addr.sin_addr.s_addr = inet_addr("10.42.0.1");
```

这里的ip填接收端的ip地址

