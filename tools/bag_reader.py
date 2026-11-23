"""Reads ROS 2 bags for the tools; needs ROS 2 sourced."""
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def bag_messages(bag_path, topics=None):
    """Yields (topic, message) in the order they were written, not by timestamp:
    WSL's clock steps backwards, so laptop timestamps can go out of order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_read_order(
        rosbag2_py.ReadOrder(sort_by=rosbag2_py.ReadOrderSortBy.File))
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, get_message(types[topic]))
