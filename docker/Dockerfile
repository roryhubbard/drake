ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND noninteractive

## drake
COPY setup/ubuntu/ setup/ubuntu/
RUN bash setup/ubuntu/install_prereqs.sh -y && rm -rf setup

# Add Docker's official GPG key:
RUN apt-get update
RUN apt-get install -y ca-certificates curl
RUN install -m 0755 -d /etc/apt/keyrings
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
RUN chmod a+r /etc/apt/keyrings/docker.asc

# Add docker repository to Apt sources:
RUN echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  tee /etc/apt/sources.list.d/docker.list > /dev/null

## Docker Outside of Docker
RUN apt-get update \
    && apt-get upgrade \
    && apt-get install -y \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin \
    systemd-coredump \
    && apt-get autoremove \
    && apt-get clean

# LLVM
# packages copied from https://apt.llvm.org/
# another option is to use a script: bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"
RUN apt-get update \
    && apt-get install -y \
    clang-format \
    clang-tidy \
    clang-tools \
    clang clangd \
    libc++-dev \
    libc++1 \
    libc++abi-dev \
    libc++abi1 \
    libclang-dev \
    libclang1 \
    liblldb-dev \
    libllvm-ocaml-dev \
    libomp-dev \
    libomp5 \
    lld \
    lldb \
    llvm-dev \
    llvm-runtime \
    llvm \
    python3-clang \
    && apt-get autoremove \
    && apt-get clean

## python libs like to call python directly
RUN ln -sf /usr/bin/python3 /usr/bin/python

CMD [ "/bin/bash" ]
