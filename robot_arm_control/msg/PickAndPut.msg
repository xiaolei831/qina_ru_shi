# 底盘运动指令消息 (对应 ROS1 pick_and_put.msg)
# car_state: 底盘状态 (0=无状态, 1=夹取完成左转, 2=放置完成右转, 3=旋转放置完成, 4=转动夹爪完成)
# angle: 底盘目标旋转角度 (rad)
float32 car_state
float32 angle
