from pathlib import Path
from rosbags.highlevel import AnyReader

bag_path = Path("data/2011-01-19-07-49-38.bag")
out_path = Path("data/base_scan.txt")

with AnyReader([bag_path]) as reader, out_path.open("w", encoding="utf-8") as out:
    connections = [c for c in reader.connections if c.topic == "/base_scan"]

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)

        timestamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        num_ranges = len(msg.ranges)

        fields = [
            str(timestamp_sec),
            str(msg.angle_min),
            str(msg.angle_max),
            str(msg.angle_increment),
            str(msg.range_min),
            str(msg.range_max),
            str(num_ranges),
        ]

        fields.extend(str(float(r)) for r in msg.ranges)

        out.write(" ".join(fields) + "\n")
