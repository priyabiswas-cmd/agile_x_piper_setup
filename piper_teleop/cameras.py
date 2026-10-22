"""Intel RealSense (D405) colour stream reader."""
import numpy as np


class RealSense:
    def __init__(self, serial, width=640, height=480, fps=30):
        import pyrealsense2 as rs   # imported here so teleop works without cameras installed
        self.serial = serial
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self.pipe.start(cfg)
        self.shape = (height, width, 3)

    def read(self):
        frames = self.pipe.wait_for_frames(timeout_ms=1000)
        return np.asanyarray(frames.get_color_frame().get_data()).copy()

    def stop(self):
        self.pipe.stop()


def list_serials():
    import pyrealsense2 as rs
    return [d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()]
