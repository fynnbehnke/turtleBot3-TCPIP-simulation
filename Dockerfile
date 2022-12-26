FROM ros:noetic
RUN apt-get update
RUN apt-get install python3 python3-pip -y
RUN apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get install ros-noetic-cv-bridge  -y
COPY requirements.txt .
RUN pip install --upgrade pip
RUN pip install -r requirements.txt
COPY . /ros_ws
WORKDIR /ros_ws
RUN bash bash_scripts/run_and_check_tests.sh
