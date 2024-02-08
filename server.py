# from sanic import Sanic
# from sanic.response import stream
# from v4l2py import Device

# app = Sanic(__name__)


# @app.route("/")
# async def index(request):
#     return response.html("""<img src="/stream">""")


# def gen_frames():
#     with Device(0) as cam:
#         for frame in cam:
#             yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame.data + b"\r\n"
#     # except OSError:
#     #     print("device is busy")


# @app.route("/stream")
# async def camera_stream(request):
#     return response.ResponseStream(
#         gen_frames, content_type="multipart/x-mixed-replace; boundary=frame"
#     )


# if __name__ == "__main__":
#     app.run("0.0.0.0", port=5000)
