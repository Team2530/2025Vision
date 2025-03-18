from flask import Flask, Response
import cv2
from queue import Queue, Empty
from threading import Thread

class VideoStream:
    def __init__(self):
        self.app = Flask(__name__)
        self.framebuffer = Queue()
        self.setup_routes()

    def publish_frame(self, frame):
        _, jpeg = cv2.imencode('.jpg', frame)
        return (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

    def setup_routes(self):
        @self.app.route('/')
        def index():
            return """
            <html>
                <head>
                    <title>Video Stream</title>
                </head>
                <body>
                    <h1>OpenCV Video Stream</h1>
                    <img src=\"/video_feed\">
                </body>
            </html>
            """

        @self.app.route('/video_feed')
        def video_feed():
            def generate_frames():
                while True:
                    try:
                        frame = self.framebuffer.get(timeout=1)
                        yield self.publish_frame(frame)
                    except Empty:
                        continue

            return Response(generate_frames(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

    def start(self):
        thread = Thread(target=self.app.run, kwargs={'debug': False, 'host': '0.0.0.0', 'port': 5000}, daemon=True)
        thread.start()

if __name__ == '__main__':
    video_stream = VideoStream()
    video_stream.start()

    capture = cv2.VideoCapture(0)
    try:
        while True:
            ret, frame = capture.read()
            if not ret:
                break
            video_stream.framebuffer.put(frame)
    finally:
        capture.release()
