# RM 控制板 USB 桥接侧 CAN 帧（与 librmcs CBoard 语义一致）
std_msgs/Header header
uint8 bus          # 1 = CAN1, 2 = CAN2
uint32 id          # 标准 11 位或扩展 29 位
string id_hex      # 非空则由此解析 ID（支持 "0x1FF"、"1FF"、十进制 "511"），优先于 id
uint8[8] data
uint8 dlc          # 1..8（勿用 0）
bool is_extended
bool is_remote
float32 periodic_rate_hz  # >0: 按该频率周期发送此帧；=0: 单次发送；<0: 停止周期发送
