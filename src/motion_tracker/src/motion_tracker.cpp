#include <motion_tracker/motion_tracker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

MotionTracker::MotionTracker() : Node("motion_tracker_node")
{
    // params
    declare_parameter("incoming_imu_data_topic", "/imu_input");
    declare_parameter("estimated_odometry_topic", "/estimated_odometry");
    declare_parameter("refresh_rate_hz", 40);

    std::string incoming_imu_data_topic, estimated_odometry_topic;
    int refresh_rate_hz;
    get_parameter("incoming_imu_data_topic", incoming_imu_data_topic);
    get_parameter("estimated_odometry_topic", estimated_odometry_topic);
    get_parameter("refresh_rate_hz", refresh_rate_hz);

    // setup
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publishStaticTransform();

    incoming_imu_data_subs_ = create_subscription<sensor_msgs::msg::Imu>(
        incoming_imu_data_topic, 50, std::bind(&MotionTracker::incomingImuDataCallback, this, std::placeholders::_1));

    estimated_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(estimated_odometry_topic, 100);
}

void MotionTracker::incomingImuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (msg)
    {
        if (estimated_odometries_.empty())
        {
            nav_msgs::msg::Odometry o;
            o.header.stamp = msg->header.stamp;
            o.pose.pose.position.x = 0.0;
            o.pose.pose.position.y = 0.0;
            o.pose.pose.position.z = 0.0;
            o.twist.twist.linear.x = 0.0;
            o.twist.twist.linear.y = 0.0;
            o.twist.twist.linear.z = 0.0;
            o.twist.twist.angular.x = 0.0;
            o.twist.twist.angular.y = 0.0;
            o.twist.twist.angular.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            o.pose.pose.orientation = tf2::toMsg(q);
            estimated_odometries_.push_back(o);
        }
        else
        {
            sensor_msgs::msg::Imu *current_imu_measurement = msg.get();
            nav_msgs::msg::Odometry *last_estimated_odometry = &estimated_odometries_.back();

            const double delta_time = (rclcpp::Time(current_imu_measurement->header.stamp) - rclcpp::Time(last_estimated_odometry->header.stamp)).seconds();

            // get previous orientation
            tf2::Quaternion q;
            tf2::fromMsg(last_estimated_odometry->pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // calc next orientation (change of angles over time)
            const double next_roll = roll + current_imu_measurement->angular_velocity.x * delta_time;   // gamma
            const double next_pitch = pitch + current_imu_measurement->angular_velocity.y * delta_time; // beta
            const double next_yaw = yaw + current_imu_measurement->angular_velocity.z * delta_time;     // alpha
            tf2::Quaternion next_q;
            next_q.setRPY(next_roll, next_pitch, next_yaw);
            next_q.normalize();

            // calc linear velocity
            const double transformed_vel_x = last_estimated_odometry->twist.twist.linear.x + (cos(next_yaw) * cos(next_pitch) * current_imu_measurement->linear_acceleration.x +
                                                                                              (cos(next_yaw) * sin(next_pitch) * sin(next_roll) - sin(next_yaw) * cos(next_roll)) * current_imu_measurement->linear_acceleration.y +
                                                                                              (cos(next_yaw) * sin(next_pitch) * cos(next_roll) + sin(next_yaw) * sin(next_roll)) * (current_imu_measurement->linear_acceleration.z)) *
                                                                                                 delta_time;
            const double transformed_vel_y = last_estimated_odometry->twist.twist.linear.y + (sin(next_yaw) * cos(next_pitch) * current_imu_measurement->linear_acceleration.x +
                                                                                              (sin(next_yaw) * sin(next_pitch) * sin(next_roll) + cos(next_yaw) * cos(next_roll)) * current_imu_measurement->linear_acceleration.y +
                                                                                              (sin(next_yaw) * sin(next_pitch) * cos(next_roll) - cos(next_yaw) * sin(next_roll)) * (current_imu_measurement->linear_acceleration.z)) *
                                                                                                 delta_time;
            const double transformed_vel_z = last_estimated_odometry->twist.twist.linear.z + ((-sin(next_pitch) * current_imu_measurement->linear_acceleration.x +
                                                                                               cos(next_pitch) * sin(next_roll) * current_imu_measurement->linear_acceleration.y +
                                                                                               cos(next_pitch) * cos(next_roll) * (current_imu_measurement->linear_acceleration.z))) *
                                                                                                 delta_time;

            // publish next estmated odometry
            nav_msgs::msg::Odometry next_estimated_odometry;
            next_estimated_odometry.header.stamp = current_imu_measurement->header.stamp;
            next_estimated_odometry.header.frame_id = "odom";
            next_estimated_odometry.child_frame_id = "base_link";

            next_estimated_odometry.pose.pose.orientation = tf2::toMsg(next_q);
            next_estimated_odometry.twist.twist.angular.x = current_imu_measurement->angular_velocity.x;
            next_estimated_odometry.twist.twist.angular.y = current_imu_measurement->angular_velocity.y;
            next_estimated_odometry.twist.twist.angular.z = current_imu_measurement->angular_velocity.z;

            next_estimated_odometry.pose.pose.position.x = last_estimated_odometry->pose.pose.position.x + transformed_vel_x * delta_time;
            next_estimated_odometry.pose.pose.position.y = last_estimated_odometry->pose.pose.position.y + transformed_vel_y * delta_time;
            next_estimated_odometry.pose.pose.position.z = last_estimated_odometry->pose.pose.position.z + transformed_vel_z * delta_time;
            next_estimated_odometry.twist.twist.linear.x = transformed_vel_x;
            next_estimated_odometry.twist.twist.linear.y = transformed_vel_y;
            next_estimated_odometry.twist.twist.linear.z = transformed_vel_z;

            estimated_odometry_pub_->publish(next_estimated_odometry);

            estimated_odometries_.push_back(next_estimated_odometry);
        }
    }
    else
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Could not retrieve IMU message");
    }
}

void MotionTracker::publishStaticTransform()
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation = tf2::toMsg(q);

    tf_static_broadcaster_->sendTransform(t);
}