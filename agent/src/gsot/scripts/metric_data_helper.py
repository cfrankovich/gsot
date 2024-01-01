#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class
import time
from functools import partial
from std_msgs.msg import String
from sensor_msgs.msg import Image


class MetricDataHelper(Node):
    def __init__(self):
        super().__init__('metric_data_helper')
        self.topic_list = []
        self.pub_buffer = {} 
        self.publisher = self.create_publisher(String, 'gsot/aggregated_data', 10)
        self.publish_timer = self.create_timer(1, self.publish_buffer)
    
    def scan_topics(self):
        current_topics = self.get_topic_names_and_types()
        for topic_name, _types in current_topics:
            if topic_name not in self.topic_list and "rosout" not in topic_name and "gsot" not in topic_name: 
                msg_type = get_msg_class(self, topic_name, include_hidden_topics=True)
                if msg_type is not None:
                    self.create_subscription(msg_type, topic_name, partial(self.generic_callback, topic_name), 1)
                    self.topic_list.append(topic_name)

    def generic_callback(self, topic_name, message):
        if isinstance(message, Image):
            self.pub_buffer[topic_name] = "<img data>" 
        else:
            fields = get_data(message) 
            self.pub_buffer[topic_name] = '\t'.join(fields) 

    def publish_buffer(self):
        sorted_topics = sorted(self.pub_buffer.keys())
        aggregated_data = '\n'.join(self.pub_buffer[topic] for topic in sorted_topics)
        self.log(aggregated_data)
        self.publisher.publish(String(data=aggregated_data))
        self.pub_buffer.clear()

    def log(self, s):
        self.get_logger().info(s)


def get_data(msg):
    result = []

    def recurse(item):
        if hasattr(item, '_fields_and_field_types'):  # Check if it's a ROS message
            for field in item._fields_and_field_types.keys():
                val = getattr(item, field)
                recurse(val)
        elif isinstance(item, (int, float, str, bool)):  # Handle simple types
            result.append(str(item))
        elif isinstance(item, list) or isinstance(item, tuple):  # Handle lists and tuples
            for elem in item:
                recurse(elem)
        elif isinstance(item, dict):  # Handle dictionaries
            for key, value in item.items():
                recurse(value)

    recurse(msg)
    return result


def main(args=None):
    rclpy.init(args=args)
    mdh = MetricDataHelper()
    mdh.log('Scanning topics...')
    for i in range(1, 11):
        mdh.scan_topics()
        mdh.log(str(11-i))
        time.sleep(1)
    mdh.log('Scan complete')
    mdh.log(f'Topics scanned: {mdh.topic_list}')
    rclpy.spin(mdh)
    mdh.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

