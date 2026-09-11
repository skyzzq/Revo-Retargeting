# Revo3 Topics

完整接口见 [Documents/接口文档.md](../Documents/接口文档.md)。日常命令见 [Documents/操作指令.md](../Documents/操作指令.md)。

```text
/manus_glove_0
/manus_glove_1
    manus_ros2_msgs/msg/ManusGlove     # 下标是发现顺序；分流看 msg.side

/revo3_{left,right}/controller_manager
/revo3_{left,right}/revo3_joint_state/joint_states
/revo3_{left,right}/joint_forward_mit_controller/retarget_targets   # 插值前
/revo3_{left,right}/joint_forward_mit_controller/commands           # 200 Hz MIT
    revo3_mit_controller_msgs/msg/Revo3MITCommand
```
