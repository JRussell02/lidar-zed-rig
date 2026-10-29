"""Small helpers around the Ouster SDK (1.0)."""
import numpy as np
from ouster.sdk import open_source


def open_osf(path):
    """Open a recording; returns (source, sensor_info)."""
    src = open_source(str(path))
    info = src.sensor_info[0] if isinstance(src.sensor_info, list) else src.sensor_info
    return src, info


def frames(src, complete_only=True):
    """Yield the single sensor's LidarFrame from each FrameSet, skipping missing/partial scans."""
    for item in src:
        fr = item[0] if hasattr(item, "valid_frames") or isinstance(item, (list, tuple)) else item
        if fr is None or (complete_only and not fr.complete()):
            continue
        yield fr


def column_host_ns(frame):
    """Host-clock time of every measurement column.

    Each UDP packet carries a block of columns and is received just after its last column is
    measured, so the offset is the median of (packet receive time - sensor time of that packet's
    last column). Requires recording with `ouster-cli ... save --ts packet`.
    """
    ts = np.asarray(frame.timestamp, dtype=np.int64)
    pkt = np.asarray(frame.packet_timestamp, dtype=np.int64)
    cpp = len(ts) // len(pkt)
    last = ts[cpp - 1::cpp]
    ok = (pkt > 0) & (last > 0)
    return ts + int(np.median(pkt[ok] - last[ok]))


def frame_start_ns(frame):
    """Host-clock arrival of the scan's first packet."""
    pkt = np.asarray(frame.packet_timestamp, dtype=np.int64)
    return int(pkt[pkt > 0].min())
