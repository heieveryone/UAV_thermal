from threading import Condition
import numpy as np
import cv2
from vidgear.gears import CamGear
from vidgear.gears import WriteGear
import serial
import threading

from seekcamera import (
    SeekCameraIOType,
    SeekCameraColorPalette,
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
    SeekCamera,
    SeekFrame,
)
import datetime

#current_time = datetime.datetime.now()
#timestamp = current_time.strftime("%Y%m%d%H%M%S")
#output_filename = f'output_{timestamp}.avi'

cap1 = cv2.VideoCapture(0)
#fourcc = cv2.VideoWriter_fourcc(*'XVID')

#define required FFmpeg parameters for your writer
output_params = {
    "-clones": ["-f", "lavfi", "-i", "anullsrc"],
    "-vcodec": "libx264",
    "-preset": "medium",
    "-b:v": "4500k",
    "-bufsize": "512k",
    "-pix_fmt": "yuv420p",
    "-f": "flv",
}

# [WARNING] Change your YouTube-Live Stream Key here:
YOUTUBE_STREAM_KEY = "rbv8-rr9z-tydm-wwyc-864t"

# Define writer with defined parameters
writer = WriteGear(
    output="rtmp://a.rtmp.youtube.com/live2/{}".format(YOUTUBE_STREAM_KEY),
    logging=True,
    **output_params
)



ser = serial.Serial('COM4', 115200, timeout= 1)

position = (700, 150)

font = cv2.FONT_HERSHEY_SIMPLEX

# 设置字体的大小和颜色
font_scale = 1
color = (0, 255, 0)

# 设置字体的粗细
thickness = 2


class Renderer:
    """Contains camera and image data required to render images to the screen."""

    def __init__(self):
        self.busy = False
        self.frame = SeekFrame()
        self.camera = SeekCamera()
        self.frame_condition = Condition()
        self.first_frame = True


def on_frame(_camera, camera_frame, renderer):
    """Async callback fired whenever a new frame is available.

    Parameters
    ----------
    _camera: SeekCamera
        Reference to the camera for which the new frame is available.
    camera_frame: SeekCameraFrame
        Reference to the class encapsulating the new frame (potentially
        in multiple formats).
    renderer: Renderer
        User defined data passed to the callback. This can be anything
        but in this case it is a reference to the renderer object.
    """

    # Acquire the condition variable and notify the main thread
    # that a new frame is ready to render. This is required since
    # all rendering done by OpenCV needs to happen on the main thread.
    with renderer.frame_condition:
        renderer.frame = camera_frame.color_argb8888
        renderer.frame_condition.notify()


def on_event(camera, event_type, event_status, renderer):
    """Async callback fired whenever a camera event occurs.

    Parameters
    ----------
    camera: SeekCamera
        Reference to the camera on which an event occurred.
    event_type: SeekCameraManagerEvent
        Enumerated type indicating the type of event that occurred.
    event_status: Optional[SeekCameraError]
        Optional exception type. It will be a non-None derived instance of
        SeekCameraError if the event_type is SeekCameraManagerEvent.ERROR.
    renderer: Renderer
        User defined data passed to the callback. This can be anything
        but in this case it is a reference to the Renderer object.
    """
    print("{}: {}".format(str(event_type), camera.chipid))

    if event_type == SeekCameraManagerEvent.CONNECT:
        if renderer.busy:
            return

        # Claim the renderer.
        # This is required in case of multiple cameras.
        renderer.busy = True
        renderer.camera = camera

        # Indicate the first frame has not come in yet.
        # This is required to properly resize the rendering window.
        renderer.first_frame = True

        # Set a custom color palette.
        # Other options can set in a similar fashion.
        camera.color_palette = SeekCameraColorPalette.TYRIAN

        # Start imaging and provide a custom callback to be called
        # every time a new frame is received.
        camera.register_frame_available_callback(on_frame, renderer)
        camera.capture_session_start(SeekCameraFrameFormat.COLOR_ARGB8888)

    elif event_type == SeekCameraManagerEvent.DISCONNECT:
        # Check that the camera disconnecting is one actually associated with
        # the renderer. This is required in case of multiple cameras.
        if renderer.camera == camera:
            # Stop imaging and reset all the renderer state.
            camera.capture_session_stop()
            renderer.camera = None
            renderer.frame = None
            renderer.busy = False

    elif event_type == SeekCameraManagerEvent.ERROR:
        print("{}: {}".format(str(event_status), camera.chipid))

    elif event_type == SeekCameraManagerEvent.READY_TO_PAIR:
        return

def NEODO():
    global NEO_data
    while True:
        hex_data = "ac350002013ef9a3"
        byte_data = bytes.fromhex(hex_data)
        # print('send command', byte_data)
        ser.write(byte_data)
        # time.sleep(0.01)
        response = ser.readline()
        # print('responses:', response)
        last_three_bytes = response[-8:-3]
        # print('measure data:', last_three_bytes)
        NEO_data = last_three_bytes.decode('utf-8', errors='ignore')
        # time.sleep(0.01)




#out = cv2.VideoWriter(output_filename, fourcc, 20, (1280, 960))
def main():
    # Create a context structure responsible for managing all connected USB cameras.
    # Cameras with other IO types can be managed by using a bitwise or of the
    # SeekCameraIOType enum cases.
    with SeekCameraManager(SeekCameraIOType.USB) as manager:
        # Start listening for events.
        renderer = Renderer()
        manager.register_event_callback(on_event, renderer)

        hex_data_cheak = "ac350002001dde93"
        byte_data_cheak = bytes.fromhex(hex_data_cheak)
        print('send command_cheak', byte_data_cheak)
        ser.write(byte_data_cheak)
        #start_device
        hex_data_start = "ac350003001e02cca2"
        byte_data_start = bytes.fromhex(hex_data_start)
        print('send command_start', byte_data_start)
        ser.write(byte_data_start)
        thread1 = threading.Thread(target=NEODO)
        thread1.start()
        while True:
            # Wait a maximum of 150ms for each frame to be received.
            # A condition variable is used to synchronize the access to the renderer;
            # it will be notified by the user defined frame available callback thread.

            ret1, img1 = cap1.read()  # 讀取第一個來源影片的每一幀
            # 裁切區域的 x 與 y 座標（左上角）
            x = 90
            y = 55

            # 裁切區域的長度與寬度
            w = 624
            h = 468

            # 裁切圖片
            #crop_img = img1[y:y + h, x:x + w]
            webcam = cv2.resize(img1, (640, 480))
            with renderer.frame_condition:
                if renderer.frame_condition.wait(150.0 / 1000.0):
                    img = renderer.frame.data

                    # Render the image to the window.
                    thermal_cam = cv2.resize(img, (640, 480))
                    webcam = cv2.copyMakeBorder(webcam, 100, 100, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
                    thermal_cam = cv2.copyMakeBorder(thermal_cam, 100, 100, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
                    webcam_bgra = cv2.cvtColor(webcam,cv2.COLOR_BGR2BGRA)
                    merged_frame = np.hstack((webcam_bgra,thermal_cam))
                    cv2.putText(merged_frame, f"NEO sensor: {NEO_data} ppm", position, font, font_scale, color, thickness)
                    cv2.imshow('Video1',merged_frame)
                    #cv2.imshow("video1", webcam)
                    #cv2.imshow("video2", thermal_cam)
                    cv2.moveWindow("Video1", 0, 0)
                    #cv2.moveWindow("video2", 640, 0)
                    writer.write(merged_frame)

            # Process key events.
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            #cap1.release()
            #out.release()
            #cv2.destoryAllWindows()
if __name__ == "__main__":
    main()