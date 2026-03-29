#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

class MlsDemoNode : public rclcpp::Node {
public:
    MlsDemoNode() : Node("mls_demo_node") {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mls_map", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MlsDemoNode::publish_mls, this));
        RCLCPP_INFO(this->get_logger(), "Đã khởi động MLS Demo Node (4 Bề mặt + Path). Đang phát...");
    }

private:
    void publish_mls() {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        // Quét ma trận không gian 10x10
        for (int x = 0; x < 10; ++x) {
            for (int y = 0; y < 10; ++y) {
                
                // 1. Mặt đất (Màu xanh lá) ở z = 0.0 (Phủ toàn bộ)
                // Đặt độ trong suốt 0.5 để dễ nhìn xuyên xuống hầm ngầm
                auto ground = create_patch(x, y, 0.0, 0.1, 0.7, 0.1, 0.5, id++); 
                marker_array.markers.push_back(ground);

                // 2. Hầm ngầm (Màu xám) ở z = -1.5 (Chạy dọc theo trục x = 2 và 3)
                if (x == 2 || x == 3) {
                    auto tunnel = create_patch(x, y, -1.5, 0.4, 0.4, 0.4, 0.9, id++);
                    marker_array.markers.push_back(tunnel);
                }

                // 3. Cầu vượt (Màu cam) ở z = 2.0 (Chạy ngang theo trục y = 6 và 7)
                if (y == 6 || y == 7) {
                    auto bridge = create_patch(x, y, 2.0, 0.9, 0.5, 0.0, 0.9, id++);
                    marker_array.markers.push_back(bridge);
                }

                // 4. Trạm quan sát (Màu đỏ) ở z = 4.0 (Nằm ngay ngã tư giao nhau)
                if ((x == 2 || x == 3) && (y == 6 || y == 7)) {
                    auto platform = create_patch(x, y, 4.0, 0.8, 0.1, 0.1, 0.9, id++);
                    marker_array.markers.push_back(platform);
                }
            }
        }

        // 5. Thêm đường đi A* mô phỏng (Màu tím)
        marker_array.markers.push_back(create_path_marker(id++));

        publisher_->publish(marker_array);
    }

    visualization_msgs::msg::Marker create_patch(float x, float y, float z, 
                                                 float r, float g, float b, float a, int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "mls_surface";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.scale.x = 0.95; marker.scale.y = 0.95; marker.scale.z = 0.1;  
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = a; 
        
        return marker;
    }

    visualization_msgs::msg::Marker create_path_marker(int id) {
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now();
        line.ns = "mls_path";
        line.id = id;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.15; // Độ dày đường

        line.color.r = 0.7; line.color.g = 0.0; line.color.b = 1.0; line.color.a = 1.0;

        auto add_pt = [&](double x, double y, double z) {
            geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = z;
            line.points.push_back(p);
        };

        // Kịch bản A*: Đất -> Xuống hầm -> Lên đất -> Lên cầu
        add_pt(8.5, 1.5, 0.2);   // Điểm Start trên mặt đất
        add_pt(2.5, 1.5, 0.2);   // Đi đến miệng hầm ngầm
        add_pt(2.5, 1.5, -1.3);  // Tụt xuống hầm ngầm
        add_pt(2.5, 8.5, -1.3);  // Đi xuyên qua hầm ngầm (dưới cầu và trạm)
        add_pt(2.5, 8.5, 0.2);   // Ngoi lên mặt đất
        add_pt(1.5, 8.5, 0.2);   // Rẽ trái trên mặt đất
        add_pt(1.5, 6.5, 0.2);   // Đi đến chân cầu
        add_pt(1.5, 6.5, 2.2);   // Leo lên cầu
        add_pt(8.5, 6.5, 2.2);   // Đi đến đầu kia của cầu (Goal)

        return line;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MlsDemoNode>());
    rclcpp::shutdown();
    return 0;
}