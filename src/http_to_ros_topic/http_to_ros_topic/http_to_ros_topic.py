import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
import requests


class HttpClient:

    def __init__(self):
        self.url = "http://192.168.1.187/get?lin_accX&lin_accY&lin_accZ&lin_acc_time&gyroX&gyroY&gyroZ&gyro_time" # url to get the latest measurement
        self.headers = {"Content-Type": "application/json"}

    def poll(self) -> dict:

        imu_data = {}

        try:
            response = requests.get(
                self.url,
                headers=self.headers,
                verify=False,
                timeout=0.1,
            )
        except requests.exceptions.RequestException:
            Node("http_to_ros_topic_").get_logger().warning("HTTP request time out.")

        else:
            if response.ok:
                json_res = response.json()
                if bool(json_res["buffer"]["lin_acc_time"]["buffer"][0]) and json_res["buffer"]["lin_acc_time"]["buffer"][0] > 0.0:
                    imu_data = {
                        "time": json_res["buffer"]["lin_acc_time"]["buffer"][0],
                        "lin_acc": {
                            "x": json_res["buffer"]["lin_accX"]["buffer"][0],
                            "y": json_res["buffer"]["lin_accY"]["buffer"][0],
                            "z": json_res["buffer"]["lin_accZ"]["buffer"][0]
                        },
                        "gyro": {
                            "x": json_res["buffer"]["gyroX"]["buffer"][0],
                            "y": json_res["buffer"]["gyroY"]["buffer"][0],
                            "z": json_res["buffer"]["gyroZ"]["buffer"][0]
                        },
                    }

        return imu_data


class RosPublisher(Node):

    def __init__(self):
        super().__init__("http_to_ros_topic")
        self.publisher_ = self.create_publisher(Imu, "/imu", 10)
        self.timer_ = self.create_timer(0.025, self.publish_imu_data_callback_)

        self.http_client = HttpClient()

    def publish_imu_data_callback_(self) -> None:

        imu_data = self.http_client.poll()

        if bool(imu_data):
            imu_msg = Imu()
            imu_msg.header.stamp = Time(seconds=0, nanoseconds=int(imu_data["time"] * 1e9)).to_msg()
            imu_msg.linear_acceleration.x = imu_data["lin_acc"]["x"]
            imu_msg.linear_acceleration.y = imu_data["lin_acc"]["y"]
            imu_msg.linear_acceleration.z = imu_data["lin_acc"]["z"]
            imu_msg.angular_velocity.x = imu_data["gyro"]["x"]
            imu_msg.angular_velocity.y = imu_data["gyro"]["y"]
            imu_msg.angular_velocity.z = imu_data["gyro"]["z"]

            self.publisher_.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(RosPublisher())

    rclpy.shutdown()


if __name__ == "__main__":
    main()
