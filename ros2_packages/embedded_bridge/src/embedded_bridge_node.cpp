#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nlohmann/json.hpp>
#include <mosquitto.h>
#include <queue>
#include <thread>
#include <mutex>

using json = nlohmann::json;

class EmbeddedBridgeNode : public rclcpp::Node {
public:
    EmbeddedBridgeNode() : Node("embedded_bridge_node") {
        // Get parameters
        this->declare_parameter("mqtt_broker", "localhost");
        this->declare_parameter("mqtt_port", 1883);
        this->declare_parameter("mqtt_qos", 1);
        
        mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
        mqtt_port_ = this->get_parameter("mqtt_port").as_int();
        mqtt_qos_ = this->get_parameter("mqtt_qos").as_int();
        
        RCLCPP_INFO(this->get_logger(), "MQTT Broker: %s:%d", 
                   mqtt_broker_.c_str(), mqtt_port_);
        
        // Initialize publishers
        heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
            "robot/embedded/heartbeat", 10);
        
        sensor_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "robot/embedded/sensors", 10);
        
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "robot/embedded/status", 10);
        
        // Initialize subscribers
        control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot/control/command", 10,
            std::bind(&EmbeddedBridgeNode::control_callback, this, std::placeholders::_1));
        
        // Initialize MQTT
        mosquitto_lib_init();
        mosq_ = mosquitto_new("ros_bridge", true, this);
        
        if (!mosq_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create mosquitto instance");
            throw std::runtime_error("Mosquitto init failed");
        }
        
        // Set callbacks
        mosquitto_connect_callback_set(mosq_, on_connect);
        mosquitto_message_callback_set(mosq_, on_message);
        mosquitto_disconnect_callback_set(mosq_, on_disconnect);
        mosquitto_log_callback_set(mosq_, on_log);
        
        // Connect to broker
        int rc = mosquitto_connect(mosq_, mqtt_broker_.c_str(), mqtt_port_, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", 
                        mosquitto_strerror(rc));
            throw std::runtime_error("MQTT connection failed");
        }
        
        // Start MQTT loop in separate thread
        mqtt_thread_ = std::thread(&EmbeddedBridgeNode::mqtt_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "Embedded Bridge initialized successfully");
    }
    
    ~EmbeddedBridgeNode() {
        mqtt_running_ = false;
        if (mqtt_thread_.joinable()) {
            mqtt_thread_.join();
        }
        
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

private:
    // MQTT variables
    mosquitto *mosq_;
    std::string mqtt_broker_;
    int mqtt_port_;
    int mqtt_qos_;
    std::thread mqtt_thread_;
    bool mqtt_running_ = true;
    std::mutex mqtt_mutex_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sensor_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
    
    // MQTT callbacks
    static void on_connect(struct mosquitto *mosq, void *userdata, int result) {
        auto self = static_cast<EmbeddedBridgeNode*>(userdata);
        
        if (result == 0) {
            RCLCPP_INFO(self->get_logger(), "MQTT connected");
            
            // Subscribe to embedded device topics
            mosquitto_subscribe(mosq, NULL, "robot/embedded/heartbeat", self->mqtt_qos_);
            mosquitto_subscribe(mosq, NULL, "robot/embedded/sensors", self->mqtt_qos_);
            mosquitto_subscribe(mosq, NULL, "robot/embedded/status", self->mqtt_qos_);
            
        } else {
            RCLCPP_ERROR(self->get_logger(), "MQTT connection failed: %s", 
                        mosquitto_connack_string(result));
        }
    }
    
    static void on_disconnect(struct mosquitto *mosq, void *userdata, int result) {
        auto self = static_cast<EmbeddedBridgeNode*>(userdata);
        RCLCPP_WARN(self->get_logger(), "MQTT disconnected: %s", 
                   mosquitto_strerror(result));
    }
    
    static void on_message(struct mosquitto *mosq, void *userdata,
                          const struct mosquitto_message *msg) {
        auto self = static_cast<EmbeddedBridgeNode*>(userdata);
        
        std::string topic(msg->topic);
        std::string payload((char*)msg->payload, msg->paylen);
        
        RCLCPP_DEBUG(self->get_logger(), "MQTT message on topic: %s", topic.c_str());
        
        std::lock_guard<std::mutex> lock(self->mqtt_mutex_);
        
        try {
            auto data = json::parse(payload);
            
            if (topic == "robot/embedded/heartbeat") {
                auto msg = std_msgs::msg::String();
                msg.data = data.dump();
                self->heartbeat_pub_->publish(msg);
                
            } else if (topic == "robot/embedded/sensors") {
                self->process_sensor_data(data);
                
            } else if (topic == "robot/embedded/status") {
                auto msg = std_msgs::msg::String();
                msg.data = data.dump();
                self->status_pub_->publish(msg);
            }
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(self->get_logger(), "JSON parse error: %s", e.what());
        }
    }
    
    static void on_log(struct mosquitto *mosq, void *userdata, int level, const char *str) {
        // Optional: log MQTT library messages
        // std::cout << "MQTT Log [" << level << "]: " << str << std::endl;
    }
    
    void mqtt_loop() {
        while (mqtt_running_) {
            {
                std::lock_guard<std::mutex> lock(mqtt_mutex_);
                int rc = mosquitto_loop(mosq_, 100, 1);
                
                if (rc == MOSQ_ERR_NO_CONN) {
                    RCLCPP_WARN(this->get_logger(), "Reconnecting to MQTT broker...");
                    mosquitto_reconnect(mosq_);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void process_sensor_data(const json &data) {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "embedded_device";
        
        // Extract sensor data
        if (data.contains("adc_raw")) {
            msg.name.push_back("adc_raw");
            msg.position.push_back(data["adc_raw"].get<double>());
        }
        
        if (data.contains("voltage")) {
            msg.name.push_back("voltage");
            msg.position.push_back(data["voltage"].get<double>());
        }
        
        if (data.contains("temperature")) {
            msg.name.push_back("temperature");
            msg.position.push_back(data["temperature"].get<double>());
        }
        
        if (data.contains("humidity")) {
            msg.name.push_back("humidity");
            msg.position.push_back(data["humidity"].get<double>());
        }
        
        sensor_pub_->publish(msg);
    }
    
    void control_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            auto cmd = json::parse(msg->data);
            
            RCLCPP_INFO(this->get_logger(), "Control command received: %s", 
                       msg->data.c_str());
            
            // Publish to MQTT
            std::string payload = cmd.dump();
            
            {
                std::lock_guard<std::mutex> lock(mqtt_mutex_);
                mosquitto_publish(mosq_, NULL, "robot/control/command", 
                                payload.length(), payload.c_str(), mqtt_qos_, false);
            }
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing control command: %s", 
                        e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<EmbeddedBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("embedded_bridge"), 
                    "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
