# web.py

import flask
from v4l2py import Device

app = flask.Flask("basic-web-cam")

cam = Device.from_id(0)
cam.open()


def gen_frames():
    try:
        for frame in cam:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame.data + b"\r\n"
    except OSError:
        print("device is busy")


@app.route("/")
def index():
    return '<html><img src="/stream" /></html>'


@app.route("/stream")
def stream():
    return flask.Response(
        gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )
