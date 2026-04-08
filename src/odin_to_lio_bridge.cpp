#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>

/**
 * OdinToLioBridge
 *
 * 订阅 odin1/cloud_raw，将字段转换为 point_lio (OUST64 模式, lidar_type=3) 期望的格式：
 *   x         float32
 *   y         float32
 *   z         float32
 *   intensity float32  (来自 odin uint8)
 *   t         uint32   (来自 odin offset_time float32，单位纳秒)
 *   ring      uint16   (行号，oust64_handler 不使用，置为行索引)
 *
 * 发布到 odin1/cloud_lio
 */
class OdinToLioBridge : public rclcpp::Node {
public:
    OdinToLioBridge() : Node("odin_to_lio_bridge") {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "odin1/cloud_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&OdinToLioBridge::convert, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "odin1/cloud_lio",
            rclcpp::SensorDataQoS());
        pub_viz_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "odin1/cloud_viz",
            rclcpp::SensorDataQoS());
        row_stride_ = declare_parameter<int>("viz_row_stride", 8);
        col_stride_ = declare_parameter<int>("viz_col_stride", 8);
        RCLCPP_INFO(get_logger(),
            "OdinToLioBridge ready: odin1/cloud_raw -> odin1/cloud_lio, "
            "odin1/cloud_viz (stride %d×%d)",
            row_stride_, col_stride_);
    }

private:
    static int field_offset(const sensor_msgs::msg::PointCloud2& msg, const std::string& name) {
        for (const auto& f : msg.fields) {
            if (f.name == name) return static_cast<int>(f.offset);
        }
        return -1;
    }

    void convert(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
        const int off_x           = field_offset(*in, "x");
        const int off_y           = field_offset(*in, "y");
        const int off_z           = field_offset(*in, "z");
        const int off_intensity   = field_offset(*in, "intensity");
        const int off_offset_time = field_offset(*in, "offset_time");

        if (off_x < 0 || off_y < 0 || off_z < 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "cloud_raw 缺少 xyz 字段，跳过本帧");
            return;
        }
        if (off_offset_time < 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "cloud_raw 缺少 offset_time 字段，t 将全为 0（无运动畸变矫正）");
        }

        auto out          = std::make_unique<sensor_msgs::msg::PointCloud2>();
        out->header       = in->header;
        out->height       = in->height;
        out->width        = in->width;
        out->is_dense     = in->is_dense;
        out->is_bigendian = false;

        // 输出点格式匹配 ouster_ros::Point（preprocess.h）：
        // x(4) y(4) z(4) intensity(4) t(4) reflectivity(2) ring(1) pad(1) ambient(2) pad(2) range(4) = 32 bytes
        using PF = sensor_msgs::msg::PointField;
        auto mf = [](const std::string& name, uint32_t offset, uint8_t type) {
            PF f;
            f.name     = name;
            f.offset   = offset;
            f.datatype = type;
            f.count    = 1;
            return f;
        };
        out->fields = {
            mf("x",            0,  PF::FLOAT32),
            mf("y",            4,  PF::FLOAT32),
            mf("z",            8,  PF::FLOAT32),
            mf("intensity",    12, PF::FLOAT32),
            mf("t",            16, PF::UINT32),
            mf("reflectivity", 20, PF::UINT16),
            mf("ring",         22, PF::UINT8),
            mf("ambient",      24, PF::UINT16),
            mf("range",        28, PF::UINT32),
        };
        constexpr uint32_t OUT_STEP = 32u;
        out->point_step = OUT_STEP;
        out->row_step   = OUT_STEP * in->width;

        const uint32_t n_pts   = in->height * in->width;
        const uint32_t in_step = in->point_step;
        out->data.assign(n_pts * OUT_STEP, 0u);

        for (uint32_t i = 0; i < n_pts; ++i) {
            const uint8_t* ip = in->data.data() + i * in_step;
            uint8_t*       op = out->data.data() + i * OUT_STEP;

            // x, y, z: float32 直接拷贝
            memcpy(op + 0, ip + off_x, 4);
            memcpy(op + 4, ip + off_y, 4);
            memcpy(op + 8, ip + off_z, 4);

            // intensity: uint8 → float32
            if (off_intensity >= 0) {
                float v = static_cast<float>(ip[off_intensity]);
                memcpy(op + 12, &v, 4);
            }

            // offset_time: float32 (纳秒) → t: uint32 (纳秒)
            if (off_offset_time >= 0) {
                float ft;
                memcpy(&ft, ip + off_offset_time, 4);
                uint32_t t = (ft >= 0.0f) ? static_cast<uint32_t>(ft) : 0u;
                memcpy(op + 16, &t, 4);
            }

            // ring: 行索引 (uint8)
            uint8_t ring = static_cast<uint8_t>((i / in->width) & 0xFF);
            op[22] = ring;

            // reflectivity, ambient, range: 留零即可（oust64_handler 不使用）
        }

        pub_->publish(std::move(out));

        // 发布降采样版本供 Foxglove 可视化
        if (pub_viz_->get_subscription_count() > 0) {
            publish_viz(*in);
        }
    }

    void publish_viz(const sensor_msgs::msg::PointCloud2& in) {
        const int off_x = field_offset(in, "x");
        const int off_y = field_offset(in, "y");
        const int off_z = field_offset(in, "z");
        if (off_x < 0 || off_y < 0 || off_z < 0) return;

        // xyz only，12 字节/点
        constexpr uint32_t VIZ_STEP = 12u;
        using PF = sensor_msgs::msg::PointField;
        auto mf = [](const std::string& name, uint32_t offset) {
            PF f; f.name = name; f.offset = offset;
            f.datatype = PF::FLOAT32; f.count = 1; return f;
        };

        auto viz = std::make_unique<sensor_msgs::msg::PointCloud2>();
        viz->header       = in.header;
        viz->height       = 1;
        viz->is_dense     = false;
        viz->is_bigendian = false;
        viz->point_step   = VIZ_STEP;
        viz->fields       = {mf("x", 0), mf("y", 4), mf("z", 8)};

        const uint32_t in_step = in.point_step;
        for (uint32_t row = 0; row < in.height; row += row_stride_) {
            for (uint32_t col = 0; col < in.width; col += col_stride_) {
                const uint8_t* ip = in.data.data() + (row * in.width + col) * in_step;
                float x, y, z;
                memcpy(&x, ip + off_x, 4);
                memcpy(&y, ip + off_y, 4);
                memcpy(&z, ip + off_z, 4);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                size_t base = viz->data.size();
                viz->data.resize(base + VIZ_STEP);
                memcpy(viz->data.data() + base + 0, &x, 4);
                memcpy(viz->data.data() + base + 4, &y, 4);
                memcpy(viz->data.data() + base + 8, &z, 4);
            }
        }
        viz->width    = viz->data.size() / VIZ_STEP;
        viz->row_step = viz->width * VIZ_STEP;
        pub_viz_->publish(std::move(viz));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_viz_;
    int row_stride_{4};
    int col_stride_{4};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdinToLioBridge>());
    rclcpp::shutdown();
    return 0;
}
