# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.209.6/containers/ubuntu/.devcontainer/base.Dockerfile

# [Choice] Ubuntu version (use hirsuite or bionic on local arm64/Apple Silicon): hirsute, focal, bionic
ARG VARIANT="hirsute"
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}

# [Optional] Uncomment this section to install additional OS packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>

RUN apt-get update -y
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends build-essential python3 python3-pip \
     make binutils gcc g++ bash gzip bzip2 tar unzip rsync wget cmake python3-lxml libxml2 \
     python3-dev python3-opencv python3-wxgtk4.0 python3-matplotlib python3-pygame

RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-a/8.3-2019.02/gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz && \
    tar -xf gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz && \
    rm gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz

# Download, build, and install CMake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.18.4/cmake-3.18.4.tar.gz && \
    tar zxvf cmake-3.18.4.tar.gz && \
    cd cmake-3.18.4 && \
    ./bootstrap && \
    make -j$(nproc) && \
    make install

#    && \
#    ln -s /usr/local/bin/cmake /usr/bin/cmake


RUN pip3 install --upgrade pip
#RUN python3 -m pip install -r $SRC/PX4-Autopilot/Tools/setup/requirements.txt
RUN python3 -m pip install argcomplete \
    argparse>=1.2 \
    cerberus \
    coverage \
    empy==3.3.4 \
    future \
    jinja2>=2.8 \
    jsonschema \
    kconfiglib \
    lxml \
    matplotlib>=3.0.* \
    numpy>=1.13 \
    nunavut>=1.1.0 \
    packaging \
    pandas>=0.21 \
    pkgconfig \
    psutil \
    pygments \
    wheel>=0.31.1 \
    pymavlink \
    pyros-genmsg \
    pyserial \
    pyulog>=0.5.0 \
    pyyaml \
    requests \
    setuptools>=39.2.0 \
    six>=1.12.0 \
    toml>=0.9 \
    sympy>=1.10.1

#RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends build-essential python3.8 python3-pip \
#     make binutils gcc g++ bash gzip bzip2 tar unzip rsync wget cmake

#RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends bison flex cpio lzop bc build-essential fakeroot lzma gettext libmpc-dev u-boot-tools \
#     python3.8 python3-pip atftp pkg-config libncurses5-dev:amd64 libssl-dev:amd64 \
#     sed make binutils gcc g++ bash patch \
#     gzip bzip2 perl tar cpio python unzip rsync wget libncurses-dev file cmake


#sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
#python3 -m pip install PyYAML mavproxy --user
#echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
