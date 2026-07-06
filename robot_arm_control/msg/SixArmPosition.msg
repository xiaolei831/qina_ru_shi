# 视觉目标位置消息。
# 当前用于 BPU 药品检测；保留 SixArmPosition 名称以兼容下游抓取流程。
# angle_x: 药品在图像中水平方向的角度偏移
# angle_y: 药品在图像中垂直方向的角度偏移
# distance: 深度相机测得的目标距离, 单位 m；无深度时为 0.0
# correct: 是否为目标药品
# color: 目标类别名称，例如 "medicine" / "class_1"
float32 angle_x
float32 angle_y
float32 distance
bool    correct
string  color
