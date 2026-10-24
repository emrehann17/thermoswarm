#!/usr/bin/env python3
"""Subscribe to real thermal images; validate metadata/rate/contrast and optionally save PNG.

No publisher, image synthesis, ARM, mode change or vehicle motion is implemented.
Without --require-hot this is a stream test (the grounded camera cannot see (1,1)).
"""
import argparse
import json
import math
from pathlib import Path
import struct
import threading
import time
import zlib

from wildfire_assets import FRAME, TOPIC


def unpack_image(width, height, encoding, step, data, stamp, frame):
    if (width, height, encoding) != (320, 240, 'mono8'):
        raise ValueError(f'Expected 320x240 mono8, received {width}x{height} {encoding}')
    if frame != FRAME:
        raise ValueError(f'Unexpected frame_id: {frame!r}, expected {FRAME!r}')
    if not math.isfinite(stamp) or stamp <= 0:
        raise ValueError(f'Invalid simulation timestamp: {stamp}')
    if step < width or len(data) != step * height:
        raise ValueError('Invalid image stride/data size')
    pixels = b''.join(bytes(data[y * step:y * step + width]) for y in range(height))
    return {'width': width, 'height': height, 'encoding': encoding,
            'step': step, 'stamp': stamp, 'frame_id': frame, 'pixels': pixels,
            'received_monotonic': time.monotonic()}


def contrast(frame):
    pixels = frame['pixels']
    hist = [0] * 256
    for v in pixels:
        hist[v] += 1
    hot = [i for i, v in enumerate(pixels) if v >= 170]
    cold = sum(hist[85:116])
    count = len(pixels)
    low, high = min(pixels), max(pixels)
    centroid = ([sum(i % frame['width'] for i in hot) / len(hot),
                 sum(i // frame['width'] for i in hot) / len(hot)] if hot else None)
    return {'min_gray': low, 'max_gray': high, 'contrast_gray': high - low,
            'temperature_range_K_approx': [low * 3, high * 3],
            'hot_pixels': len(hot), 'hot_fraction': len(hot) / count,
            'ambient_pixels': cold, 'hot_centroid_px': centroid,
            'hot_visible': len(hot) >= 20 and len(hot) < .7 * count and cold >= 100 and high - low >= 60}


def write_png(path, frame):
    """Lossless grayscale PNG from received bytes, without scaling or invented pixels."""
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data) & 0xffffffff)
    w, h = frame['width'], frame['height']
    raw = b''.join(b'\0' + frame['pixels'][i * w:(i + 1) * w] for i in range(h))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', w, h, 8, 0, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def acceptance(report, require_hot, min_hz):
    """Keep transport success separate from actual hot-region evidence."""
    stream_ok = report['arrival_hz'] >= min_hz
    hotspot_ok = stream_ok and report['hot_visible']
    return {'thermal_check_pass': stream_ok, 'hotspot_detected_pass': hotspot_ok,
            'hotspot_required': require_hot, 'pass': hotspot_ok if require_hot else stream_ok}


def capture(transport, count, timeout, topic=TOPIC):
    frames, errors = [], []
    lock = threading.Lock()
    def accept(*args):
        try:
            frame = unpack_image(*args)
            with lock:
                if len(frames) < count:
                    frames.append(frame)
        except ValueError as exc:
            errors.append(str(exc))
    if transport == 'gz':
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image, PixelFormatType
        node = Node()
        def callback(msg):
            frame = next((d.value[0] for d in msg.header.data if d.key == 'frame_id' and d.value), '')
            encoding = 'mono8' if PixelFormatType.Name(msg.pixel_format_type) == 'L_INT8' else PixelFormatType.Name(msg.pixel_format_type)
            accept(msg.width, msg.height, encoding, msg.step, msg.data,
                   msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, frame)
        node.subscribe(Image, topic, callback)
        spin = lambda: time.sleep(.02)
        close = lambda: node.unsubscribe(topic)
    else:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        rclpy.init()
        node = rclpy.create_node('thermoswarm_thermal_check')
        def callback(msg):
            accept(msg.width, msg.height, msg.encoding, msg.step, msg.data,
                   msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.header.frame_id)
        node.create_subscription(Image, topic, callback, qos_profile_sensor_data)
        spin = lambda: rclpy.spin_once(node, timeout_sec=.05)
        def close():
            node.destroy_node()
            rclpy.shutdown()
    try:
        deadline = time.monotonic() + timeout
        while len(frames) < count and not errors and time.monotonic() < deadline:
            spin()
    finally:
        close()
    if errors:
        raise ValueError('; '.join(errors[:3]))
    if len(frames) < count:
        raise TimeoutError(f'{transport}: received {len(frames)}/{count} real frames in {timeout}s; no placeholder generated')
    stamps = [f['stamp'] for f in frames]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('Non-increasing image timestamps')
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transport', choices=('gz', 'ros'), default='ros')
    parser.add_argument('--frames', type=int, default=16)
    parser.add_argument('--timeout', type=float, default=30)
    parser.add_argument('--require-hot', action='store_true')
    parser.add_argument('--min-hz', type=float, default=0, help='Minimum arrival rate in wall time (0: report only)')
    parser.add_argument('--output', type=Path, help='Save genuine grayscale PNG and adjacent JSON report')
    args = parser.parse_args()
    if (args.frames < 2 or not math.isfinite(args.timeout) or args.timeout <= 0
            or not math.isfinite(args.min_hz) or args.min_hz < 0):
        parser.error('frames >= 2, timeout > 0, min-hz >= 0 required')
    frames = capture(args.transport, args.frames, args.timeout)
    frame = max(frames, key=lambda f: contrast(f)['contrast_gray'])
    report = {k: v for k, v in frame.items() if k not in ('pixels', 'received_monotonic')}
    report.update(contrast(frame))
    report.update({'transport': args.transport, 'topic': TOPIC, 'frames_received': len(frames),
                   'simulation_hz': (len(frames) - 1) / (frames[-1]['stamp'] - frames[0]['stamp']),
                   'arrival_hz': (len(frames) - 1) / (frames[-1]['received_monotonic'] - frames[0]['received_monotonic']),
                   'source': 'live sensor subscription; no synthetic image'})
    report.update(acceptance(report, args.require_hot, args.min_hz))
    if args.output:
        write_png(args.output, frame)
        args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    print('THERMAL CHECK ' + ('PASS' if report['thermal_check_pass'] else 'FAIL') + ' (transport/metadata/rate only)')
    if args.require_hot:
        print('HOTSPOT DETECTED ' + ('PASS' if report['hotspot_detected_pass'] else 'FAIL'))
    else:
        print('HOTSPOT ACCEPTANCE NOT REQUESTED (use --require-hot or thermal_watch.py)')
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, TimeoutError, ImportError) as exc:
        print(f'THERMAL CHECK FAIL: {exc}')
        raise SystemExit(1)
