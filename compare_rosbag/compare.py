#!/usr/bin/env python3
"""
ROS2 Bag Comparison Tool

Compares two rosbag files (.db3 or .mcap) for specified topics.
Checks if all messages from bag1 are contained in bag2.
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Any, Set
import yaml
import hashlib

# Import rosbag2 readers
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except ImportError as e:
    print(f"Error: Required ROS2 packages not found: {e}")
    print("Please ensure ROS2 is installed and sourced.")
    sys.exit(1)


class RosbagReader:
    """Reads messages from rosbag files (.db3 or .mcap)"""

    def __init__(self, bag_path: str):
        """
        Initialize rosbag reader

        Args:
            bag_path: Path to rosbag file or directory
        """
        self.bag_path = Path(bag_path)
        self.reader = SequentialReader()

        # Determine storage format
        storage_id = self._get_storage_id()

        # Set up storage options
        storage_options = StorageOptions(
            uri=str(self.bag_path),
            storage_id=storage_id
        )

        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        try:
            self.reader.open(storage_options, converter_options)
        except Exception as e:
            raise RuntimeError(f"Failed to open rosbag at {bag_path}: {e}")

    def _get_storage_id(self) -> str:
        """Determine storage format based on file extension"""
        if self.bag_path.is_dir():
            # Check for .db3 files in directory
            if list(self.bag_path.glob("*.db3")):
                return "sqlite3"
            # Check for .mcap files
            elif list(self.bag_path.glob("*.mcap")):
                return "mcap"
        elif self.bag_path.is_file():
            suffix = self.bag_path.suffix
            if suffix == ".db3":
                return "sqlite3"
            elif suffix == ".mcap":
                return "mcap"

        # Default to sqlite3
        return "sqlite3"

    def get_topic_info(self) -> Dict[str, Any]:
        """Get information about topics in the bag"""
        topic_types = self.reader.get_all_topics_and_types()
        return {t.name: t.type for t in topic_types}

    def read_all_message_hashes(self, topics: List[str]) -> Dict[str, Set[str]]:
        """
        Read all messages from specified topics and compute their hashes in a single pass.
        This is memory efficient for large messages (e.g., images, point clouds).
        Uses SHA-256 hash of binary data for comparison.

        Args:
            topics: List of topic names to read

        Returns:
            Dictionary mapping topic names to sets of message hashes
        """
        # Initialize result dictionary with sets (for O(1) lookup)
        hashes_by_topic = {topic: set() for topic in topics}

        # Build set of topics we're interested in
        topic_set = set(topics)

        # Read all messages in a single pass and compute hashes
        while self.reader.has_next():
            topic_name, data, _ = self.reader.read_next()

            # Only process topics we're interested in
            if topic_name in topic_set:
                # Hash the binary data (CDR serialized)
                # This is memory efficient for large messages
                msg_hash = hashlib.sha256(bytes(data)).hexdigest()
                hashes_by_topic[topic_name].add(msg_hash)

        return hashes_by_topic

    def read_all_messages_with_hashes(self, topics: List[str]) -> Dict[str, List[tuple]]:
        """
        Read all messages with their hashes (for bag1 to track which messages are missing).
        Returns message index and hash pairs.

        Args:
            topics: List of topic names to read

        Returns:
            Dictionary mapping topic names to lists of (index, hash) tuples
        """
        # Initialize result dictionary
        messages_by_topic = {topic: [] for topic in topics}

        # Build set of topics we're interested in
        topic_set = set(topics)

        # Track message count per topic
        topic_counters = {topic: 0 for topic in topics}

        # Read all messages in a single pass
        while self.reader.has_next():
            topic_name, data, _ = self.reader.read_next()

            # Only process topics we're interested in
            if topic_name in topic_set:
                # Hash the binary data with index
                msg_hash = hashlib.sha256(bytes(data)).hexdigest()
                messages_by_topic[topic_name].append((topic_counters[topic_name], msg_hash))
                topic_counters[topic_name] += 1

        return messages_by_topic

    def close(self):
        """Close the reader"""
        del self.reader


class BagComparator:
    """Compares two rosbag files"""

    def __init__(self, bag1_path: str, bag2_path: str, config: Dict[str, Any]):
        """
        Initialize comparator

        Args:
            bag1_path: Path to first rosbag
            bag2_path: Path to second rosbag
            config: Configuration dictionary
        """
        self.bag1_path = bag1_path
        self.bag2_path = bag2_path
        self.config = config
        self.topics = config.get('topics', [])
        self.comparison_opts = config.get('comparison', {})

    def compare(self) -> bool:
        """
        Compare the two rosbags
        Checks if all messages from bag1 are contained in bag2

        Returns:
            True if all messages from bag1 exist in bag2, False otherwise
        """
        print(f"\n{'='*60}")
        print(f"Comparing rosbags:")
        print(f"  Bag 1: {self.bag1_path}")
        print(f"  Bag 2: {self.bag2_path}")
        print(f"{'='*60}\n")

        # Read all topics from both bags in a single pass (memory optimized)
        # bag1: read with indices to track missing messages
        # bag2: read as hash set for O(1) lookup (memory efficient for large messages)
        print("Reading all topics from bag 1...")
        reader1 = RosbagReader(self.bag1_path)
        messages1_all = reader1.read_all_messages_with_hashes(self.topics)
        reader1.close()

        print("Reading all topics from bag 2...")
        reader2 = RosbagReader(self.bag2_path)
        hashes2_all = reader2.read_all_message_hashes(self.topics)
        reader2.close()

        print()

        # Compare each topic
        all_contained = True
        for topic in self.topics:
            print(f"\nChecking topic: {topic}")
            print("-" * 40)

            contained = self._compare_topic(
                topic,
                messages1_all.get(topic, []),
                hashes2_all.get(topic, set())
            )
            if not contained:
                all_contained = False

        print(f"\n{'='*60}")
        if all_contained:
            print("✓ All messages from bag1 are contained in bag2!")
        else:
            print("✗ Some messages from bag1 are NOT in bag2")
        print(f"{'='*60}\n")

        return all_contained

    def _compare_topic(self, _topic: str, messages1_with_hashes: List[tuple], hashes2: Set[str]) -> bool:
        """
        Compare a single topic between two bags using hash-based comparison.
        Checks if all messages from bag1 are contained in bag2.
        Uses SHA-256 hash of binary data for memory-efficient comparison.

        Args:
            _topic: Topic name (unused, for clarity)
            messages1_with_hashes: List of (index, hash) tuples from bag1
            hashes2: Set of message hashes from bag2
        """
        # Check if all messages from bag1 exist in bag2 (using hash lookup - O(1))
        missing_messages = []

        for idx, msg_hash in messages1_with_hashes:
            if msg_hash not in hashes2:
                missing_messages.append(idx)

        if missing_messages:
            print(f"  ✗ {len(missing_messages)} messages from bag1 NOT found in bag2")
            if self.comparison_opts.get('show_differences', True):
                max_show = self.comparison_opts.get('max_differences_to_show', 10)
                for idx in missing_messages[:max_show]:
                    print(f"    Message #{idx} from bag1 not in bag2")
                if len(missing_messages) > max_show:
                    print(f"    ... and {len(missing_messages) - max_show} more")
            return False
        else:
            print(f"  ✓ All {len(messages1_with_hashes)} messages from bag1 are contained in bag2")

        return True


def load_config(config_path: str) -> Dict[str, Any]:
    """Load configuration from YAML file"""
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"Error loading config file {config_path}: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description='Compare two ROS2 bag files for specified topics'
    )
    parser.add_argument(
        'bag1',
        help='Path to first rosbag file'
    )
    parser.add_argument(
        'bag2',
        help='Path to second rosbag file'
    )
    parser.add_argument(
        '-c', '--config',
        default='config.yaml',
        help='Path to configuration file (default: config.yaml)'
    )

    args = parser.parse_args()

    # Validate inputs
    if not Path(args.bag1).exists():
        print(f"Error: Bag 1 not found: {args.bag1}")
        sys.exit(1)

    if not Path(args.bag2).exists():
        print(f"Error: Bag 2 not found: {args.bag2}")
        sys.exit(1)

    if not Path(args.config).exists():
        print(f"Error: Config file not found: {args.config}")
        sys.exit(1)

    # Load configuration
    config = load_config(args.config)

    # Run comparison
    comparator = BagComparator(args.bag1, args.bag2, config)
    match = comparator.compare()

    # Exit with appropriate code
    sys.exit(0 if match else 1)


if __name__ == '__main__':
    main()
