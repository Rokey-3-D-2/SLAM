#Anomalyreport.msg

string anomaly_label         # 감지된 이상 종류 (ex: "스크래치", "떨어짐")
string description           # 설명 메시지 (ex: "Error Detected : 스크래치")

float32 robot_x
float32 robot_y
float32 robot_z

float32 object_x
float32 object_y
float32 object_z

builtin_interfaces/Time timestamp

sensor_msgs/Image image