#!/usr/bin/env python3
"""
Multi-topic publisher from rosbag.

Extract multiple topics from a rosbag at a specified timestamp and publish them.

IMPORTANT: You must source your ROS2 workspace before running this script:
    source ~/pilot-auto/install/setup.bash
    python3 publish_topics.py --config config.yaml
"""

import argparse
import logging
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Tuple

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
except ImportError as e:
    print(f"Error: Required ROS2 packages not found: {e}")
    print("Please ensure you have sourced your ROS2 workspace and installed rosbag2_py")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class MultiTopicPublisher(Node):
    """ROS2 Node for publishing multiple topics"""

    def __init__(self):
        super().__init__('multi_topic_publisher')
        self._topic_publishers: Dict[str, Tuple[any, any]] = {}  # topic_name -> (publisher, msg_type)

    def create_topic_publisher(self, topic_name: str, msg_type_name: str):
        """Create a publisher for a specific topic"""
        msg_type = get_message(msg_type_name)
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        publisher = self.create_publisher(msg_type, topic_name, qos_profile)
        self._topic_publishers[topic_name] = (publisher, msg_type)
        logger.info(f"Created publisher for topic: {topic_name} (type: {msg_type_name})")

    def publish_message(self, topic_name: str, serialized_data: bytes):
        """Deserialize and publish message to a specific topic"""
        if topic_name not in self._topic_publishers:
            logger.error(f"Publisher for topic {topic_name} not found")
            return False

        publisher, msg_type = self._topic_publishers[topic_name]
        msg = deserialize_message(serialized_data, msg_type)
        publisher.publish(msg)
        return True


def parse_timestamp(timestamp_str: str) -> int:
    """
    Parse timestamp string to nanoseconds.

    Supports:
    - Nanoseconds: "1234567890123456789"
    - Seconds with decimal: "1234567890.123"

    Args:
        timestamp_str: Timestamp string

    Returns:
        Timestamp in nanoseconds
    """
    try:
        # Try parsing as float (seconds)
        if '.' in str(timestamp_str) or 'e' in str(timestamp_str).lower():
            seconds = float(timestamp_str)
            return int(seconds * 1e9)
        # Try parsing as integer (nanoseconds)
        else:
            timestamp_ns = int(timestamp_str)
            # If the number is too small, assume it's in seconds
            if timestamp_ns < 1e15:  # Before year 2001 if in nanoseconds
                return int(timestamp_ns * 1e9)
            return timestamp_ns
    except ValueError:
        raise ValueError(f"Invalid timestamp format: {timestamp_str}")


def find_closest_message(rosbag_path: Path, topic_name: str, target_timestamp: int):
    """
    Find the message closest to the target timestamp on a specific topic.

    Args:
        rosbag_path: Path to rosbag file (.db3 or .mcap)
        topic_name: Topic name to search
        target_timestamp: Target timestamp in nanoseconds

    Returns:
        Tuple of (message_data, msg_type_name, timestamp) or None if not found
    """
    # Determine storage format
    if rosbag_path.suffix == '.db3':
        storage_id = 'sqlite3'
        uri = str(rosbag_path)
    elif rosbag_path.suffix == '.mcap':
        storage_id = 'mcap'
        uri = str(rosbag_path)
    else:
        logger.error(f"Unsupported file format: {rosbag_path.suffix}")
        return None

    # Set up storage options
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    try:
        # Open rosbag for reading
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        topic_type_map = {t.name: t.type for t in topic_types}

        if topic_name not in topic_type_map:
            logger.error(f"Topic '{topic_name}' not found in rosbag")
            logger.info(f"Available topics: {list(topic_type_map.keys())}")
            return None

        msg_type_name = topic_type_map[topic_name]
        logger.info(f"Searching for topic: {topic_name} (type: {msg_type_name})")

        # Search for closest message
        closest_message = None
        closest_timestamp = None
        min_diff = float('inf')
        message_count = 0

        # Set topic filter
        storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
        reader.set_filter(storage_filter)

        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            message_count += 1

            diff = abs(timestamp - target_timestamp)
            if diff < min_diff:
                min_diff = diff
                closest_message = data
                closest_timestamp = timestamp

        logger.info(f"Scanned {message_count} messages on topic '{topic_name}'")

        if closest_message is None:
            logger.error(f"No messages found on topic '{topic_name}'")
            return None

        time_diff_sec = min_diff / 1e9
        logger.info(f"Found closest message at timestamp: {closest_timestamp} ns (diff: {time_diff_sec:.6f}s)")

        return (closest_message, msg_type_name, closest_timestamp)

    except Exception as e:
        logger.error(f"Error reading rosbag: {e}", exc_info=True)
        return None


def load_config(config_path: Path) -> dict:
    """Load configuration from YAML file"""
    if not config_path.exists():
        logger.error(f"Config file not found: {config_path}")
        sys.exit(1)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Expand home directory in rosbag_path
    if 'rosbag_path' in config:
        config['rosbag_path'] = Path(config['rosbag_path']).expanduser()

    return config


def main():
    parser = argparse.ArgumentParser(
        description="Publish multiple topics from rosbag at a specified timestamp",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
IMPORTANT: Source your ROS2 workspace first!
  source ~/pilot-auto/install/setup.bash

Examples:
  # Basic usage
  %(prog)s /path/to/rosbag.mcap 1234567890.5 --config config.yaml

  # Test without publishing
  %(prog)s /path/to/rosbag.mcap 1234567890.5 --config config.yaml --no-publish
        """
    )
    parser.add_argument(
        "rosbag_path",
        type=Path,
        help="Path to rosbag file (.db3 or .mcap)"
    )
    parser.add_argument(
        "timestamp",
        type=str,
        help="Target timestamp (seconds with decimal or nanoseconds)"
    )
    parser.add_argument(
        "-c", "--config",
        type=Path,
        default=Path(__file__).parent / "config.yaml",
        help="Path to YAML configuration file (default: ./config.yaml)"
    )
    parser.add_argument(
        "--no-publish",
        action="store_true",
        help="Only find messages without publishing (for testing)"
    )

    args = parser.parse_args()

    # Load configuration
    logger.info(f"Loading configuration from: {args.config}")
    config = load_config(args.config)

    # Get rosbag path from command line argument
    rosbag_path = args.rosbag_path.expanduser().resolve()
    if not rosbag_path.exists():
        logger.error(f"Rosbag file not found: {rosbag_path}")
        sys.exit(1)

    logger.info(f"Rosbag file: {rosbag_path}")

    # Get target timestamp from command line argument
    try:
        target_timestamp = parse_timestamp(args.timestamp)
        logger.info(f"Target timestamp: {target_timestamp} ns ({args.timestamp})")
    except ValueError as e:
        logger.error(str(e))
        sys.exit(1)

    # Get topics to publish
    topics = config.get('topics', [])
    if not topics:
        logger.error("No topics specified in config")
        sys.exit(1)

    logger.info(f"Processing {len(topics)} topic(s)")

    # Extract messages from rosbag
    topic_messages = []
    for topic_config in topics:
        source_topic = topic_config.get('source_topic')
        publish_topic = topic_config.get('publish_topic', source_topic)

        if not source_topic:
            logger.warning("Topic config missing source_topic, skipping")
            continue

        logger.info(f"\n--- Extracting: {source_topic} ---")
        result = find_closest_message(rosbag_path, source_topic, target_timestamp)

        if result:
            message_data, msg_type_name, found_timestamp = result
            topic_messages.append({
                'source_topic': source_topic,
                'publish_topic': publish_topic,
                'message_data': message_data,
                'msg_type_name': msg_type_name,
                'timestamp': found_timestamp
            })
        else:
            logger.error(f"Failed to extract message from topic: {source_topic}")

    if not topic_messages:
        logger.error("No messages were extracted")
        sys.exit(1)

    logger.info(f"\nSuccessfully extracted {len(topic_messages)} message(s)")

    # Publish messages if not disabled
    if args.no_publish:
        logger.info("Skipping publish (--no-publish specified)")
        logger.info("Done")
        return

    # Get publish options
    publish_options = config.get('publish_options', {})
    num_publishes = publish_options.get('num_publishes', 5)
    publish_interval = publish_options.get('publish_interval', 0.1)
    connection_wait_time = publish_options.get('connection_wait_time', 1.0)

    logger.info(f"\n--- Publishing {len(topic_messages)} topic(s) ---")
    logger.info(f"Publish count: {num_publishes}, Interval: {publish_interval}s")

    try:
        # Initialize ROS2
        rclpy.init()

        # Create multi-topic publisher node
        node = MultiTopicPublisher()

        # Create publishers for all topics
        for topic_msg in topic_messages:
            node.create_topic_publisher(
                topic_msg['publish_topic'],
                topic_msg['msg_type_name']
            )

        # Wait for subscribers to connect
        logger.info(f"Waiting {connection_wait_time}s for subscribers to connect...")
        time.sleep(connection_wait_time)

        # Publish all messages multiple times
        logger.info(f"Publishing all messages {num_publishes} times...")
        for i in range(num_publishes):
            for topic_msg in topic_messages:
                node.publish_message(
                    topic_msg['publish_topic'],
                    topic_msg['message_data']
                )
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(publish_interval)
            logger.info(f"Published round {i+1}/{num_publishes}")

        logger.info("Successfully published all messages")

        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

    except Exception as e:
        logger.error(f"Error during publishing: {e}", exc_info=True)
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
        sys.exit(1)

    logger.info("Done")


if __name__ == "__main__":
    main()
