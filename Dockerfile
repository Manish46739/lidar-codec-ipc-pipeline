FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    zip \
    gdb \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
