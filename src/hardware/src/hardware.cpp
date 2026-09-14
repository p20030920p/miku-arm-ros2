/*
    【硬件】机械臂上下位机通信
    1.串口16进制通讯
    2.集成订阅与发布功能

*/

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>  // 串口库（本工作区提供 POSIX 兼容实现，见 include/serial/serial.h）
#include <arm_control/msg/arm_msg.hpp>  // 自定义结构体消息头文件
#include <cstdio>   // printf / fflush
#include <cmath>    // M_PI / fabs
#include <string>

// ============================================================
// 状态/模式 数字 → 中文（供 HUD 显示）
// ============================================================
const char* statusToStr(int s)
{
    switch (s) {
        case 0:  return "暂停";
        case 1:  return "运行";
        case 2:  return "复位";
        default: return "未知";
    }
}

const char* modeToStr(int m)
{
    switch (m) {
        case 1:  return "MIT";
        case 2:  return "位置模式";
        case 3:  return "速度模式";
        default: return "未知";
    }
}

void printHUD(const arm_control::msg::ArmMsg& m, int gripper_protect,
              bool serial_ok, const std::string& serial_error);

std::vector<uint8_t> serial_buffer;  // 持久化的串口数据缓冲区
const size_t RX_PACKET_SIZE = 46;  // 上行反馈帧
const size_t TX_PACKET_SIZE = 50;  // 下行指令帧

// 创建共用体 转换 高低八位 与 16进制整数
typedef union{
    int16_t value;
    struct{
        uint8_t low;
        uint8_t high;
    } bytes;
} RegisterUnion;

RegisterUnion kp_rx;
RegisterUnion kd_rx;

RegisterUnion pos_1_rx;
RegisterUnion vel_1_rx;
RegisterUnion tor_1_rx;

RegisterUnion pos_2_rx;
RegisterUnion vel_2_rx;
RegisterUnion tor_2_rx;

RegisterUnion pos_3_rx;
RegisterUnion vel_3_rx;
RegisterUnion tor_3_rx;

RegisterUnion pos_4_rx;
RegisterUnion vel_4_rx;
RegisterUnion tor_4_rx;

RegisterUnion pos_5_rx;
RegisterUnion vel_5_rx;
RegisterUnion tor_5_rx;

RegisterUnion pos_6_rx;
RegisterUnion vel_6_rx;
RegisterUnion tor_6_rx;

RegisterUnion pos_7_rx;
RegisterUnion vel_7_rx;
RegisterUnion tor_7_rx;

RegisterUnion kp_tx;
RegisterUnion kd_tx;

RegisterUnion pos_1_tx;
RegisterUnion vel_1_tx;
RegisterUnion tor_1_tx;

RegisterUnion pos_2_tx;
RegisterUnion vel_2_tx;
RegisterUnion tor_2_tx;

RegisterUnion pos_3_tx;
RegisterUnion vel_3_tx;
RegisterUnion tor_3_tx;

RegisterUnion pos_4_tx;
RegisterUnion vel_4_tx;
RegisterUnion tor_4_tx;

RegisterUnion pos_5_tx;
RegisterUnion vel_5_tx;
RegisterUnion tor_5_tx;

RegisterUnion pos_6_tx;
RegisterUnion vel_6_tx;
RegisterUnion tor_6_tx;

RegisterUnion pos_7_tx;
RegisterUnion vel_7_tx;
RegisterUnion tor_7_tx;

// ============================================================
// 夹爪（电机7）安全保护参数
// ============================================================
constexpr double GRIPPER_POS_UPPER_RAD =  0.00;
constexpr double GRIPPER_POS_LOWER_RAD = -165.0 * M_PI / 180.0;    // -1.597 rad
constexpr double GRIPPER_MAX_TORQUE    =  1.50;                    // N·m, 力矩安全上限
constexpr double GRIPPER_TORQUE_HYST   =  0.05;                    // N·m, 滞回释放阈值
constexpr double GRIPPER_POS_MARGIN    =  0.50  * M_PI / 180.0;    // 跑偏容忍 0.5°

const int GRIPPER_OK          = 0;   // 正常
const int GRIPPER_TORQUE_BIT  = 1;   // bit0：回读力矩过载 
const int GRIPPER_POS_BIT     = 2;   // bit1：回读位置越界

class SerialNode : public rclcpp::Node {
private:
    serial::Serial  ser;              // 串口对象
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;   // 订阅者
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr    pub;   // 发布者
    
    // 自定义结构体消息
    arm_control::msg::ArmMsg received_msg;
    arm_control::msg::ArmMsg send_msg;

    // 夹爪安全状态（位掩码，见 GRIPPER_*_BIT）
    int gripper_protect_ = GRIPPER_OK;

    // 串口连接状态
    bool serial_ok_    = true;   // 串口是否正常
    std::string serial_error_ = "";   // 最近一次串口错误描述

    // 串口设备与波特率（由节点参数覆盖；默认值 = 原来硬编码的实机值）
    std::string serial_port_   = "/dev/ttyACM0";
    int         serial_baudrate_ = 115200;

    // 单线程 executor：主循环里手动 spin_some()，等价于 ROS 1 的 ros::spinOnce()
    rclcpp::executors::SingleThreadedExecutor executor_;

public:
    // 成员函数：统一配置并打开串口
    // ROS 2 差异：原 ROS 1 版本把 "/dev/ttyACM0" 与 115200 写死；这里改为节点参数
    //   serial_port（默认 /dev/ttyACM0）、serial_baudrate（默认 115200）。
    //   实机行为完全不变（默认值就是原来的值），但使得仿真/测试可以把串口指向
    //   虚拟串口（如 socat 创建的 PTY），从而在没有真实驱动板的情况下测试本节点。
    void openSerial()
    {
        ser.setPort(serial_port_);
        ser.setBaudrate(serial_baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }

    SerialNode() : rclcpp::Node("serial_communication_node") {
        // 串口参数（默认值与原来的硬编码一致）
        serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        serial_baudrate_ = this->declare_parameter<int>("serial_baudrate", 115200);

        // 初始化发布者订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_rx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
            "Arm_tx", 20,
            std::bind(&SerialNode::msgCallback, this, std::placeholders::_1));

        bool error_reported = false; // 串口错误只提示一次
        // 串口初始化
        while (ser.isOpen()!= true)
        {
            try {
                openSerial(); 
            } catch (serial::IOException& e) {
                if (!error_reported) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port, retrying...");
                    error_reported = true;
                }
            }
        }
        
        if(ser.isOpen()){
            serial_ok_ = true;  
            serial_error_ = "";
            RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port initialized");
        } else {
            serial_ok_=false; 
            serial_error_="打开失败，重试中";
            RCLCPP_ERROR_STREAM(this->get_logger(), "Serial Port failed to initialize");
        }
    }

    // 订阅消息回调函数
    void msgCallback(const arm_control::msg::ArmMsg::SharedPtr msg) {
        // printf("订阅到消息");

        // 将接收到的结构体转为待发送数据
        send_msg = *msg;

        // printf("状态：%d 模式：%d 位置1:%.2f 速度1:%.2f 转矩1:%.2f 位置2:%.2f 速度2:%.2f 转矩2:%.2f",
        //     send_msg.task_status,send_msg.mode,send_msg.pos_1,send_msg.vel_1,send_msg.tor_1,send_msg.pos_2,send_msg.vel_2,send_msg.tor_2);

        // 设置缓冲区
        uint8_t buffer[50]={0x86,0xc1};

        // -------------------------------
        // 订阅数据处理 并 发送给下位机
        // -------------------------------
        buffer[2] = send_msg.task_status;  // 机械臂 运行状态 0-暂停/1-运行/2-复位
        buffer[3] = send_msg.mode;         // 机械臂 运行模式 1-mit/2-pos/3-spd

        // 电机1 参数
        pos_1_tx.value = send_msg.pos_1*1000; // 位置 rad
        vel_1_tx.value = send_msg.vel_1*1000; // 速度 rad/s
        tor_1_tx.value = send_msg.tor_1*1000; // 转矩 N*m
        buffer[4] = pos_1_tx.bytes.high; // 位置 高八位
        buffer[5] = pos_1_tx.bytes.low;  // 位置 低八位
        buffer[6] = vel_1_tx.bytes.high; // 速度 高八位
        buffer[7] = vel_1_tx.bytes.low;  // 速度 低八位
        buffer[8] = tor_1_tx.bytes.high; // 转矩 高八位
        buffer[9] = tor_1_tx.bytes.low;  // 转矩 低八位

        // 电机2 参数
        pos_2_tx.value = send_msg.pos_2*1000; // 位置 rad
        vel_2_tx.value = send_msg.vel_2*1000; // 速度 rad/s
        tor_2_tx.value = send_msg.tor_2*1000; // 转矩 N*m
        buffer[10] = pos_2_tx.bytes.high; // 位置 高八位
        buffer[11] = pos_2_tx.bytes.low;  // 位置 低八位
        buffer[12] = vel_2_tx.bytes.high; // 速度 高八位
        buffer[13] = vel_2_tx.bytes.low;  // 速度 低八位
        buffer[14] = tor_2_tx.bytes.high; // 转矩 高八位
        buffer[15] = tor_2_tx.bytes.low;  // 转矩 低八位

        // 电机3 参数
        pos_3_tx.value = send_msg.pos_3*1000; // 位置 rad
        vel_3_tx.value = send_msg.vel_3*1000; // 速度 rad/s
        tor_3_tx.value = send_msg.tor_3*1000; // 转矩 N*m
        buffer[16] = pos_3_tx.bytes.high; // 位置 高八位
        buffer[17] = pos_3_tx.bytes.low;  // 位置 低八位
        buffer[18] = vel_3_tx.bytes.high; // 速度 高八位
        buffer[19] = vel_3_tx.bytes.low;  // 速度 低八位
        buffer[20] = tor_3_tx.bytes.high; // 转矩 高八位
        buffer[21] = tor_3_tx.bytes.low;  // 转矩 低八位

        // 电机4 参数
        pos_4_tx.value = send_msg.pos_4*1000; // 位置 rad
        vel_4_tx.value = send_msg.vel_4*1000; // 速度 rad/s
        tor_4_tx.value = send_msg.tor_4*1000; // 转矩 N*m
        buffer[22] = pos_4_tx.bytes.high; // 位置 高八位
        buffer[23] = pos_4_tx.bytes.low;  // 位置 低八位
        buffer[24] = vel_4_tx.bytes.high; // 速度 高八位
        buffer[25] = vel_4_tx.bytes.low;  // 速度 低八位
        buffer[26] = tor_4_tx.bytes.high; // 转矩 高八位
        buffer[27] = tor_4_tx.bytes.low;  // 转矩 低八位

        // 电机5 参数
        pos_5_tx.value = send_msg.pos_5*1000; // 位置 rad
        vel_5_tx.value = send_msg.vel_5*1000; // 速度 rad/s
        tor_5_tx.value = send_msg.tor_5*1000; // 转矩 N*m
        buffer[28] = pos_5_tx.bytes.high; // 位置 高八位
        buffer[29] = pos_5_tx.bytes.low;  // 位置 低八位
        buffer[30] = vel_5_tx.bytes.high; // 速度 高八位
        buffer[31] = vel_5_tx.bytes.low;  // 速度 低八位
        buffer[32] = tor_5_tx.bytes.high; // 转矩 高八位
        buffer[33] = tor_5_tx.bytes.low;  // 转矩 低八位

        // 电机6 参数
        pos_6_tx.value = send_msg.pos_6*1000; // 位置 rad
        vel_6_tx.value = send_msg.vel_6*1000; // 速度 rad/s
        tor_6_tx.value = send_msg.tor_6*1000; // 转矩 N*m
        buffer[34] = pos_6_tx.bytes.high; // 位置 高八位
        buffer[35] = pos_6_tx.bytes.low;  // 位置 低八位
        buffer[36] = vel_6_tx.bytes.high; // 速度 高八位
        buffer[37] = vel_6_tx.bytes.low;  // 速度 低八位
        buffer[38] = tor_6_tx.bytes.high; // 转矩 高八位
        buffer[39] = tor_6_tx.bytes.low;  // 转矩 低八位

        // --- 电机7（夹爪）参数 ---
        pos_7_tx.value = send_msg.pos_7 * 1000;
        vel_7_tx.value = send_msg.vel_7 * 1000;
        tor_7_tx.value = send_msg.tor_7 * 1000;
        buffer[40] = pos_7_tx.bytes.high;
        buffer[41] = pos_7_tx.bytes.low;
        buffer[42] = vel_7_tx.bytes.high;
        buffer[43] = vel_7_tx.bytes.low;
        buffer[44] = tor_7_tx.bytes.high;
        buffer[45] = tor_7_tx.bytes.low;

        // kp kd 参数 — 偏移量从 40/42 改为 46/48
        kp_tx.value = send_msg.kp * 1000;
        kd_tx.value = send_msg.kd * 1000;
        buffer[46] = kp_tx.bytes.high;
        buffer[47] = kp_tx.bytes.low;
        buffer[48] = kd_tx.bytes.high;
        buffer[49] = kd_tx.bytes.low;


        // ============================================================
        // 夹爪安全保护（TX 侧）
        // ============================================================

        // --- 机制一：位置限幅 ---
        double safe_pos_7 = send_msg.pos_7;
        if (safe_pos_7 > GRIPPER_POS_UPPER_RAD) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "[夹爪安全] 目标位置 %.2f° 超上限，已钳位至 %.2f°",
                            send_msg.pos_7 / M_PI * 180, GRIPPER_POS_UPPER_RAD / M_PI * 180);
            safe_pos_7 = GRIPPER_POS_UPPER_RAD;
        }
        if (safe_pos_7 < GRIPPER_POS_LOWER_RAD) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "[夹爪安全] 目标位置 %.2f° 超下限，已钳位至 %.2f°",
                            send_msg.pos_7 / M_PI * 180, GRIPPER_POS_LOWER_RAD / M_PI * 180);
            safe_pos_7 = GRIPPER_POS_LOWER_RAD;
        }

        // 重填电机7位置缓冲区（用安全值覆盖）
        pos_7_tx.value = static_cast<int16_t>(safe_pos_7 * 1000);
        buffer[40] = pos_7_tx.bytes.high;
        buffer[41] = pos_7_tx.bytes.low;

        // --- 机制二：力矩过载保护（钳制下发） ---
        if (gripper_protect_ != GRIPPER_OK){
            // 保护激活：强制力矩为 0，pos 锁定回读位置
            tor_7_tx.value = 0;
            buffer[44] = 0;
            buffer[45] = 0;

            pos_7_tx.value = static_cast<int16_t>(received_msg.pos_7 * 1000);
            buffer[40] = pos_7_tx.bytes.high;
            buffer[41] = pos_7_tx.bytes.low;

            // 日志（可选，精确描述）
            // ROS_WARN_THROTTLE(0.5, "[夹爪安全] 保护激活: %s%s",
            //     (gripper_protect_ & GRIPPER_TORQUE_BIT) ? "力矩过载 " : "",
            //     (gripper_protect_ & GRIPPER_POS_BIT)    ? "位置越界"  : "");
        }

        // 发送给下位机（加 try-catch，写失败不崩溃）
        try{ 
            ser.write(buffer, 50); 
            serial_ok_ = true; 
            serial_error_ = ""; 
        }
        // ROS 2 差异（实为修正）：serial::IOException 继承自 SerialException，
        // 原 ROS 1 代码把 SerialException 写在前面，导致下面的 IOException
        // 分支永远不可达（g++ -Wexceptions 会告警）。这里调换顺序，
        // 使"设备消失"能如实报成"写入IO异常"。
        catch(serial::IOException& e){ 
            serial_ok_ = false; 
            serial_error_ = "写入IO异常"; 
        }
        catch(serial::SerialException& e){ 
            serial_ok_ = false; 
            serial_error_ = "写入异常"; 
        }
        catch(...){ 
            serial_ok_ = false; 
            serial_error_ = "写入未知异常"; 
        }

        // printf(" ");
        // printf("发送数据：");
        // printf("位置1:%d 位置2:%d 位置3:%d 位置4:%d 位置5:%d 位置6:%d"
        //         ,pos_1_tx.value,pos_2_tx.value,pos_3_tx.value,pos_4_tx.value,pos_5_tx.value,pos_6_tx.value);

        // printf("%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x ",
        //     buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],
        //     buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11],
        //     buffer[12],buffer[13],buffer[14],buffer[15]
        // );
        // printf("%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x ",
        //     buffer[16],buffer[17],buffer[18],buffer[19],buffer[20],buffer[21],
        //     buffer[22],buffer[23],buffer[24],buffer[25],buffer[26],buffer[27],
        //     buffer[28],buffer[129],buffer[30],buffer[31]
        // );
        // printf("%x %x %x %x %x %x %x %x %x %x %x %x %x",
        //     buffer[32],buffer[33],buffer[34],buffer[35],buffer[36],buffer[37],
        //     buffer[38],buffer[39],buffer[40],buffer[41],buffer[42],buffer[43],
        //     buffer[44]
        // );
    }

    // 把本节点挂到内部 executor（必须在 run() 之前调用一次）
    void attachExecutor()
    {
        executor_.add_node(this->shared_from_this());
    }

    // 主运行循环
    void run() {
        // 说明：这个外层 recv_buffer 在原 ROS 1 代码里就没被用过——真正取包的
        // 是下面循环内部同名的局部数组（用 std::copy 填的）。为保持与原工程
        // 一致，这里保留该声明，只是显式标注"故意未使用"以消除编译告警。
        uint8_t recv_buffer[RX_PACKET_SIZE] = {0};  // 接收缓冲区
        (void)recv_buffer;   // 与原工程一致：外层缓冲区未被使用
        int hud_tick = 0;
        
        while(rclcpp::ok()) {
            // =====================================================
            // 第 1 步：确保串口已打开（每次循环都尝试，直到成功）
            // =====================================================
            if (!ser.isOpen())
            {
                try {
                    openSerial();               // 尝试重新打开串口
                    serial_ok_    = true;       // 打开成功
                    serial_error_ = "";
                    RCLCPP_WARN(this->get_logger(), "[串口] 重连成功");
                }
                catch (...) {
                    serial_ok_    = false;      // 打开失败
                    serial_error_ = "重连失败，等待设备恢复...";
                    usleep(500000);             // 退避 0.5 秒，避免空转刷屏
                }
            }

            // =====================================================
            // 第 2 步：读串口（只有已打开才读，避免未打开时抛异常）
            // =====================================================
            if (ser.isOpen())
            {
                try {
                    if (ser.available() > 0){
                        size_t available = ser.available();
                        std::vector<uint8_t> temp_buffer(available);
                        size_t bytes_read = ser.read(temp_buffer.data(), available);
                        serial_buffer.insert(serial_buffer.end(),
                                            temp_buffer.begin(), temp_buffer.begin() + bytes_read);
                    }
                    serial_ok_    = true;       // 本次读写正常 → 连接正常
                    serial_error_ = "";
                }
                catch (serial::IOException& e) {
                    // 注意：IOException 继承 SerialException，必须放在前面！
                    // 单片机复位/拔USB → 关闭串口，下轮循环自动重连
                    serial_ok_    = false;
                    serial_error_ = "IO异常(可能单片机复位)";
                    serial_buffer.clear();
                    try { ser.close(); }        // 关闭（失败也不管）
                    catch (...) {}
                }
                catch (serial::SerialException& e) {
                    serial_ok_    = false;
                    serial_error_ = "读取异常(可能断开)";
                    serial_buffer.clear();
                    try { ser.close(); }
                    catch (...) {}
                }
                catch (...) {
                    serial_ok_    = false;
                    serial_error_ = "未知异常";
                    serial_buffer.clear();
                    try { ser.close(); }
                    catch (...) {}
                }
            }

            // =====================================================
            // 第 3 步：解析持久化缓冲区中的数据（以下保持原样）
            // =====================================================
            bool packet_processed = false;
            (void)packet_processed;   // 与原工程一致：只置位、未读取（保留原逻辑）
            while (true) {
                // 查找帧头
                size_t start_pos = 0;
                bool found_header = false;
                // 确保缓冲区足够长以包含可能的帧头
                while (start_pos <= serial_buffer.size() - 2 && serial_buffer.size() >= RX_PACKET_SIZE) {
                    if (serial_buffer[start_pos] == 0x86 && serial_buffer[start_pos + 1] == 0xc2) {
                        found_header = true;
                        break;
                    }
                    start_pos++;
                }

                // 检查是否有完整数据包
                if (found_header && (serial_buffer.size() - start_pos) >= RX_PACKET_SIZE) {
                    // 提取数据包
                    uint8_t recv_buffer[RX_PACKET_SIZE];
                    std::copy(&serial_buffer[start_pos], &serial_buffer[start_pos + RX_PACKET_SIZE], recv_buffer);

                    // -------------------------------
                    // 接收下位机数据 并 发布数据处理
                    // -------------------------------
                    if(recv_buffer[0] == 0x86 && recv_buffer[1] == 0xc2){
                        received_msg.task_status = recv_buffer[2]; // 机械臂 状态 0-暂停/1-运行/2-复位
                        received_msg.mode        = recv_buffer[3]; // 机械臂 运动模式

                        // 电机1 参数
                        pos_1_rx.bytes.high      = recv_buffer[4];
                        pos_1_rx.bytes.low       = recv_buffer[5];  
                        vel_1_rx.bytes.high      = recv_buffer[6];
                        vel_1_rx.bytes.low       = recv_buffer[7];  
                        tor_1_rx.bytes.high      = recv_buffer[8];
                        tor_1_rx.bytes.low       = recv_buffer[9];  
                        received_msg.pos_1       = pos_1_rx.value/1000.f;     // 位置 rad
                        received_msg.vel_1       = vel_1_rx.value/1000.f;     // 速度 rad/s
                        received_msg.tor_1       = tor_1_rx.value/1000.f;     // 转矩

                        // 电机2 参数
                        pos_2_rx.bytes.high      = recv_buffer[10];
                        pos_2_rx.bytes.low       = recv_buffer[11];  
                        vel_2_rx.bytes.high      = recv_buffer[12];
                        vel_2_rx.bytes.low       = recv_buffer[13];  
                        tor_2_rx.bytes.high      = recv_buffer[14];
                        tor_2_rx.bytes.low       = recv_buffer[15];  
                        received_msg.pos_2       = pos_2_rx.value/1000.f;     // 位置 rad
                        received_msg.vel_2       = vel_2_rx.value/1000.f;     // 速度 rad/s
                        received_msg.tor_2       = tor_2_rx.value/1000.f;     // 转矩

                        // 电机3 参数
                        pos_3_rx.bytes.high      = recv_buffer[16];
                        pos_3_rx.bytes.low       = recv_buffer[17];  
                        vel_3_rx.bytes.high      = recv_buffer[18];
                        vel_3_rx.bytes.low       = recv_buffer[19];  
                        tor_3_rx.bytes.high      = recv_buffer[20];
                        tor_3_rx.bytes.low       = recv_buffer[21];  
                        received_msg.pos_3       = pos_3_rx.value/1000.f;     // 位置 rad
                        received_msg.vel_3       = vel_3_rx.value/1000.f;     // 速度 rad/s
                        received_msg.tor_3       = tor_3_rx.value/1000.f;     // 转矩

                        // 电机4 参数
                        pos_4_rx.bytes.high      = recv_buffer[22];
                        pos_4_rx.bytes.low       = recv_buffer[23];  
                        vel_4_rx.bytes.high      = recv_buffer[24];
                        vel_4_rx.bytes.low       = recv_buffer[25];  
                        tor_4_rx.bytes.high      = recv_buffer[26];
                        tor_4_rx.bytes.low       = recv_buffer[27];  
                        received_msg.pos_4       = pos_4_rx.value/1000.f;     // 位置 rad
                        received_msg.vel_4       = vel_4_rx.value/1000.f;     // 速度 rad/s
                        received_msg.tor_4       = tor_4_rx.value/1000.f;     // 转矩

                        // 电机5 参数
                        pos_5_rx.bytes.high      = recv_buffer[28];
                        pos_5_rx.bytes.low       = recv_buffer[29];  
                        vel_5_rx.bytes.high      = recv_buffer[30];
                        vel_5_rx.bytes.low       = recv_buffer[31];  
                        tor_5_rx.bytes.high      = recv_buffer[32];
                        tor_5_rx.bytes.low       = recv_buffer[33];  
                        received_msg.pos_5       = pos_5_rx.value/1000.f;     // 位置 rad
                        received_msg.vel_5       = vel_5_rx.value/1000.f;     // 速度 rad/s
                        received_msg.tor_5       = tor_5_rx.value/1000.f;     // 转矩

                        // 电机6 参数
                        pos_6_rx.bytes.high      = recv_buffer[34];
                        pos_6_rx.bytes.low       = recv_buffer[35];  
                        vel_6_rx.bytes.high      = recv_buffer[36];
                        vel_6_rx.bytes.low       = recv_buffer[37];  
                        tor_6_rx.bytes.high      = recv_buffer[38];
                        tor_6_rx.bytes.low       = recv_buffer[39];  
                        received_msg.pos_6       = pos_6_rx.value/1000.f;     // 位置 rad
                        received_msg.vel_6       = vel_6_rx.value/1000.f;     // 速度 rad/s
                        received_msg.tor_6       = tor_6_rx.value/1000.f;     // 转矩

                        // 电机7（夹爪）参数
                        pos_7_rx.bytes.high      = recv_buffer[40];
                        pos_7_rx.bytes.low       = recv_buffer[41];
                        vel_7_rx.bytes.high      = recv_buffer[42];
                        vel_7_rx.bytes.low       = recv_buffer[43];
                        tor_7_rx.bytes.high      = recv_buffer[44];
                        tor_7_rx.bytes.low       = recv_buffer[45];
                        received_msg.pos_7       = pos_7_rx.value / 1000.f;
                        received_msg.vel_7       = vel_7_rx.value / 1000.f;
                        received_msg.tor_7       = tor_7_rx.value / 1000.f;

                        // ============================================================
                        // 夹爪安全检测（RX 侧）
                        // ============================================================

                       // ---- 机制二：力矩过载（滞回） ----
                        double abs_torque = fabs(received_msg.tor_7);
                        if (abs_torque > GRIPPER_MAX_TORQUE)
                        {
                            gripper_protect_ |= GRIPPER_TORQUE_BIT;        // 置位：力矩过载
                        }
                        else if (abs_torque < GRIPPER_TORQUE_HYST)
                        {
                            gripper_protect_ &= ~GRIPPER_TORQUE_BIT;       // 清位：力矩回落
                        }

                        // ---- 机制三：位置越界 ----
                        if (received_msg.pos_7 > GRIPPER_POS_UPPER_RAD + GRIPPER_POS_MARGIN ||
                            received_msg.pos_7 < GRIPPER_POS_LOWER_RAD - GRIPPER_POS_MARGIN)
                        {
                            gripper_protect_ |= GRIPPER_POS_BIT;           // 置位：位置越界
                        }
                        else
                        {
                            gripper_protect_ &= ~GRIPPER_POS_BIT;          // 清位：位置正常
                        }
                    }

                    // 发布处理后的消息
                    pub->publish(received_msg);
                    // printf("已发布消息\n");

                    // 移除已处理的数据（包括帧头之前可能存在的垃圾数据）
                    serial_buffer.erase(serial_buffer.begin(), serial_buffer.begin() + start_pos + RX_PACKET_SIZE);
                    packet_processed = true;
                } else {
                    // 找不到帧头时清理垃圾
                    if (serial_buffer.size() >= RX_PACKET_SIZE) {
                        // 丢弃最前面一个字节，尝试重新同步
                        serial_buffer.erase(serial_buffer.begin());
                        continue;  // 重新尝试查找帧头
                    }
                    break;
                }
            }
            
            // 替换原来那一大段 printf 刷屏，改成 10Hz HUD
            if (++hud_tick % 10 == 0)      // 100Hz 循环 → 每 10 次刷新一次 = 10Hz
            {
                printHUD(received_msg, gripper_protect_, serial_ok_, serial_error_);
            }
            // 处理ROS回调（单线程 executor 手动泵一次，等价于 ros::spinOnce()）
            executor_.spin_some();
            // 控制循环频率
            usleep(10000);  // 10ms
        }
    }
};

// ============================================================
// 终端 HUD：原地刷新参数（替代刷屏）
// 原理：\033[2J 清屏  +  \033[H 光标回左上角  +  重画固定行
// ============================================================
void printHUD(const arm_control::msg::ArmMsg& m, int gripper_protect,
              bool serial_ok, const std::string& serial_error){
    printf("\033[2J\033[H");

    // ===== 第 1 行：串口连接状态 =====
    if (serial_ok)
        printf("\033[32m [串口] 已连接 \033[0m\n");
    else
        printf("\033[31m [串口] 断开/异常: %s \033[0m\n", serial_error.c_str());

    printf(" 【机械臂】状态:%s  模式:%s\n", statusToStr(m.task_status), modeToStr(m.mode));
    printf("--------------------------------------------\n");
    printf(" 电机 |   位置(°)   |   速度   |   转矩\n");
    printf("--------------------------------------------\n");
    printf("  1   | %8.2f  | %8.4f | %8.4f\n", m.pos_1/M_PI*180, m.vel_1, m.tor_1);
    printf("  2   | %8.2f  | %8.4f | %8.4f\n", m.pos_2/M_PI*180, m.vel_2, m.tor_2);
    printf("  3   | %8.2f  | %8.4f | %8.4f\n", m.pos_3/M_PI*180, m.vel_3, m.tor_3);
    printf("  4   | %8.2f  | %8.4f | %8.4f\n", m.pos_4/M_PI*180, m.vel_4, m.tor_4);
    printf("  5   | %8.2f  | %8.4f | %8.4f\n", m.pos_5/M_PI*180, m.vel_5, m.tor_5);
    printf("  6   | %8.2f  | %8.4f | %8.4f\n", m.pos_6/M_PI*180, m.vel_6, m.tor_6);
    printf("  7   | %8.2f  | %8.4f | %8.4f  (夹爪)\n", m.pos_7/M_PI*180, m.vel_7, m.tor_7);
    printf("--------------------------------------------\n");

    // ===== 报错行区 =====
    if (!serial_ok)                                          // 串口报错（异常时）
        printf("\033[31m [串口] %s \033[0m\n", serial_error.c_str());

    if (gripper_protect == GRIPPER_OK)                       // 夹爪安全
        printf("\033[32m [夹爪安全] 正常 \033[0m\n");
    else
    {
        printf("\033[31m [夹爪安全] 保护激活: ");
        if (gripper_protect & GRIPPER_TORQUE_BIT) printf("力矩过载 ");
        if (gripper_protect & GRIPPER_POS_BIT)    printf("位置越界");
        printf("\033[0m\n");
    }

    printf("============================================\n");
    fflush(stdout);
}

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    node->attachExecutor();   // 把 executor 绑到本节点
    node->run();
    rclcpp::shutdown();
    return 0;
}