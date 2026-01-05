# backend/bag/reader.py
"""
Bag file reading utilities.
"""
import os
from typing import Dict, List

BAG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "public", "bag_files"))
SPOOFED_BAG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "public", "spoofed_bags"))


def topic_map(reader) -> Dict[str, str]:
    """Get a mapping of topic names to their types from a bag reader."""
    out = {}
    for md in reader.get_all_topics_and_types():
        out[md.name] = md.type
    return out


def norm_ns(ns: str) -> str:
    """Normalize a namespace to always start with /."""
    if not ns:
        return ""
    return ns if ns.startswith("/") else ("/" + ns)


def list_bag_files() -> List[Dict]:
    """List bag files from both bag_files/ and spoofed_bags/ directories."""
    os.makedirs(BAG_DIR, exist_ok=True)
    os.makedirs(SPOOFED_BAG_DIR, exist_ok=True)
    out = []

    # List from bag_files/ (both .mcap files and directories)
    for name in sorted(os.listdir(BAG_DIR)):
        p = os.path.join(BAG_DIR, name)
        size = 0
        try:
            if os.path.isdir(p):
                # Directory-style bag (MCAP2)
                size = sum(
                    os.path.getsize(os.path.join(p, f))
                    for f in os.listdir(p)
                    if os.path.isfile(os.path.join(p, f))
                )
            elif name.endswith(".mcap"):
                size = os.path.getsize(p)
            else:
                continue
        except OSError:
            pass
        out.append({"name": name, "size": size, "spoofed": False})

    # List from spoofed_bags/
    for name in sorted(os.listdir(SPOOFED_BAG_DIR)):
        p = os.path.join(SPOOFED_BAG_DIR, name)
        if not os.path.isdir(p):
            continue
        size = 0
        try:
            size = sum(
                os.path.getsize(os.path.join(p, f))
                for f in os.listdir(p)
                if os.path.isfile(os.path.join(p, f))
            )
        except OSError:
            pass
        out.append({"name": name, "size": size, "spoofed": True})

    return out
