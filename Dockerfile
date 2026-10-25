FROM ubuntu:18.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools, OpenCV 3.2.0, and Ceres dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    pkg-config \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# Download, compile, and install Ceres Solver 2.1.0 system-wide
RUN curl -sSL https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz | tar -xz \
    && mkdir -p ceres-bin \
    && cd ceres-bin \
    && cmake ../ceres-solver-2.1.0 \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
    && make -j$(nproc) \
    && make install \
    && cd .. \
    && rm -rf ceres-solver-2.1.0 ceres-bin

# Set workspace directory
WORKDIR /workspace

# Copy repository source files into /workspace
COPY . .

# Default command drops user into the build environment shell
CMD ["/bin/bash"]