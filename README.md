# robot

## Setup
```bash
python3 -m venv .venv
soruce .venv/bin/activate
which python
python3 -m pip --version
```

## Installation
```bash
python3 -m pip install -r requirements.txt
```

## Video stream server
For the video stream to work you need to start `cheese` as `pi` user.
Then start the server with:

```bash
./run_server
```

## Docker 
```bash
docker build -t test/ros:app .
docker run -it --rm test/ros:app
```