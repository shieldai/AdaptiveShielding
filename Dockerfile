FROM ubuntu:bionic

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update
RUN apt-get -y install apt-utils sudo nano
RUN apt-get -y install openssh-server

# INSTALL STORM
ENV STORM_DIR /opt/storm/
RUN apt-get -y install build-essential git cmake libboost-all-dev libcln-dev libgmp-dev libginac-dev \
    automake libglpk-dev libhwloc-dev libz3-dev libxerces-c-dev libeigen3-dev

RUN git clone -b stable https://github.com/moves-rwth/storm.git $STORM_DIR

RUN cd $STORM_DIR \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make

ENV PATH $PATH:$STORM_DIR/build/bin

# SET storm binary path
RUN ln -s $STORM_DIR/build/bin/storm /usr/bin/storm

# INSTALL SUMO
ENV SUMO_HOME=/usr/share/sumo
RUN apt-get -y update
RUN apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:sumo/stable
RUN apt-get -y update
RUN apt-get -y install sumo sumo-tools sumo-doc

# INSTALL PYTHON
COPY requirements.txt /temp/
RUN apt-get -y install python3-pip
RUN apt-get -y install python3-tk

RUN python3 -m pip install --upgrade pip

# https://stable-baselines.readthedocs.io/en/master/guide/install.html
RUN apt-get -y install libopenmpi-dev python3-dev zlib1g-dev
RUN python3 -m pip install opencv-python
RUN apt-get -y install -y libsm6 libxext6
RUN apt-get -y install -y libxrender-dev

RUN python3 -m pip install -r /temp/requirements.txt

RUN apt-get -y install libncurses-dev
RUN apt-get -y install g++ gdb gdbserver
