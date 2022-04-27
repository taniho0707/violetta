FROM ubuntu:20.04
WORKDIR /work

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    build-essential \
    git \
    bzip2 \
    wget \
    zlib1g-dev \
    clang \
    llvm \
    lld \
    cpio \
    libncurses5 \
    libncurses-dev \
    libncursesw5 && \
    apt-get clean
COPY ./gcc-arm-11.2-2022.02-x86_64-arm-none-eabi.tar.xz /work/
RUN tar -xf gcc-arm-11.2-2022.02-x86_64-arm-none-eabi.tar.xz && \
    rm gcc-arm-11.2-2022.02-x86_64-arm-none-eabi.tar.xz

RUN wget -q https://www.python.org/ftp/python/3.6.8/Python-3.6.8.tar.xz && \
    tar -xf Python-3.6.8.tar.xz && \
    cd Python-3.6.8 && \
    ./configure --enable-shared --enable-optimizations && \
    make altinstall && \
    cd /work && \
    ln /usr/local/lib/libpython3.6m.so.1.0 /usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0 && \
    rm -R Python-3.6.8 && \
    rm Python-3.6.8.tar.xz

ENV PATH "/work/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/bin:$PATH"

# RUN ln -s /usr/bin/ld.lld /usr/bin/ld
