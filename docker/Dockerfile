ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG HOME
ARG GID
ARG UID
ARG USERNAME

#RUN userdel -r ubuntu # only for ubuntu >= 24.04

ENV DEBIAN_FRONTEND noninteractive

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

RUN apt-get update \
    && apt-get upgrade \
    && apt-get install -y \
    # Docker Outside of Docker
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin \
    # Other stuff
    protobuf-compiler \
    python3-pip \
    python3-venv \
    sudo \
    systemd-coredump \
    vim \
    wget \
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

# Rust
ENV RUSTUP_HOME=${HOME}/.rustup
ENV CARGO_HOME=${HOME}/.cargo
RUN curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH=$PATH:${CARGO_HOME}/bin

# Golang
RUN wget https://go.dev/dl/go1.22.5.linux-amd64.tar.gz && \
    rm -rf /usr/local/go && \
    tar -C /usr/local -xzf go1.22.5.linux-amd64.tar.gz
ENV PATH=$PATH:/usr/local/go/bin

# Google Chrome
RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb \
    && apt-get install -y ./google-chrome-stable_current_amd64.deb \
    && rm google-chrome-stable_current_amd64.deb

# Python
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install --user pipx \
    && python3 -m pipx ensurepath \
    && python3 -m pipx ensurepath --global \
    && python3 -m pipx install cmake conan uv

# Bake user in
RUN groupadd --non-unique --gid $GID $USERNAME
RUN useradd --home-dir $HOME \
	--gid $GID \
	--uid $UID \
	--password $(openssl passwd -1 $USERNAME) \
	--groups sudo \
	--shell /bin/bash \
	$USERNAME
RUN chown --recursive --changes $USERNAME:$USERNAME $HOME

# drake
COPY setup/ubuntu/ setup/ubuntu/
RUN bash setup/ubuntu/install_prereqs.sh -y && rm -rf setup

# apt update and clean
RUN apt-get update \
    && apt-get autoremove \
    && apt-get upgrade \
    && apt-get clean

ENV IN_DOCKER_CONTAINER=1

CMD [ "/bin/bash" ]
