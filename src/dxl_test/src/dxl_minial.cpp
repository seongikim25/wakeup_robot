#include <rclcpp/rclcpp.hpp>
#include <dynamixel_workbench_toolbox/dynamixel_workbench_toolbox/dynamixel_workbench.h>
using namespace std::chrono_literals;

class DxlTestNode : public rclcpp::Node
{
public:
    DxlTestNode() : Node("dxl_test_node")
    {
        const char* device_name = "/dev/ttyUSB0";
        uint32_t baudrate = 57600;
        
        if (!dxl_.init(device_name, baudrate)) {
            RCLCPP_ERROR(get_logger(), "Dynamixel init 실패");
            rclcpp::shutdown();
            return;
        }

      
        uint8_t id = 1;
        const char* log = nullptr;
        
        // 1️⃣ PING
        uint16_t model = 0;
        if (!dxl_.ping(id, &model, &log)) {
            RCLCPP_ERROR(get_logger(), "PING 실패 (ID %d): %s", id, log ? log : "no log");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "PING 성공 ID %d, model=%u", id, model);
        
        // 2️⃣ Position Limit 확인
        int32_t min_pos = 0, max_pos = 0;
        if (dxl_.itemRead(id, "Min_Position_Limit", &min_pos, &log)) {
            RCLCPP_INFO(get_logger(), "Min Position Limit: %d", min_pos);
        }
        if (dxl_.itemRead(id, "Max_Position_Limit", &max_pos, &log)) {
            RCLCPP_INFO(get_logger(), "Max Position Limit: %d", max_pos);
        }
        
        // 3️⃣ Torque OFF (Limit 변경을 위해)
        if (!dxl_.torqueOff(id)) {
            RCLCPP_WARN(get_logger(), "Torque OFF 실패");
        }
        
        // 4️⃣ Position Limit 재설정 (XL430 범위: 0~4095)
        if (!dxl_.itemWrite(id, "Min_Position_Limit", 0, &log)) {
            RCLCPP_ERROR(get_logger(), "Min Limit 설정 실패: %s", log ? log : "no log");
        } else {
            RCLCPP_INFO(get_logger(), "Min Position Limit -> 0");
        }
        
        if (!dxl_.itemWrite(id, "Max_Position_Limit", 4095, &log)) {
            RCLCPP_ERROR(get_logger(), "Max Limit 설정 실패: %s", log ? log : "no log");
        } else {
            RCLCPP_INFO(get_logger(), "Max Position Limit -> 4095");
        }
        
        // 5️⃣ LED ON
        if (!dxl_.ledOn(id)) {
            RCLCPP_ERROR(get_logger(), "LED ON 실패");
        } else {
            RCLCPP_INFO(get_logger(), "LED ON");
        }
        
        // 6️⃣ Torque ON
        if (!dxl_.torqueOn(id)) {
            RCLCPP_ERROR(get_logger(), "Torque ON 실패");
            rclcpp::shutdown();
            return;
        } else {
            RCLCPP_INFO(get_logger(), "Torque ON");
        }
        
        // 7️⃣ 현재 위치 확인
        int32_t present_pos = 0;
        if (dxl_.itemRead(id, "Present_Position", &present_pos, &log)) {
            RCLCPP_INFO(get_logger(), "현재 위치: %d", present_pos);
        }
        
        // 8️⃣ Goal Position (중앙값)
        int32_t goal = 2048;
        if (!dxl_.goalPosition(id, goal, &log)) {
            RCLCPP_ERROR(get_logger(), "Goal Position 실패: %s", log ? log : "no log");
        } else {
            RCLCPP_INFO(get_logger(), "ID %d -> %d 이동 명령 전송 성공!", id, goal);
        }
        
        // 9️⃣ 잠시 대기 후 위치 확인
        rclcpp::sleep_for(2s);
        
        if (dxl_.itemRead(id, "Present_Position", &present_pos, &log)) {
            RCLCPP_INFO(get_logger(), "2초 후 위치: %d (목표: %d)", present_pos, goal);
        }
        
        RCLCPP_INFO(get_logger(), "테스트 노드 완료");
    }
    
private:
    DynamixelWorkbench dxl_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DxlTestNode>());
    rclcpp::shutdown();
    return 0;
}
