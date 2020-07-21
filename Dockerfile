FROM ros:kinetic
SHELL ["/bin/bash", "-c"]
ENV TINI_VERSION v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini
RUN apt update && apt upgrade -y && \
    apt install -y git gcc make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev && \
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> /root/.bashrc && \
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> /root/.bashrc && \
    source /opt/ros/kinetic/setup.bash && \
    rosdep update && \
    mkdir -p /opt/rosws/src && \
    cd /opt/rosws/src && \
    catkin_init_workspace && \
    cd /opt/rosws && \
    catkin_make && \
    echo 'source /opt/ros/kinetic/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/rosws/devel/setup.bash' >> /root/.bashrc && \
    git clone https://github.com/pyenv/pyenv.git /root/.pyenv && \
    /root/.pyenv/bin/pyenv init - && \
    /root/.pyenv/bin/pyenv install 3.8.3 && \
    /root/.pyenv/bin/pyenv global 3.8.3
WORKDIR /opt/rosws
ENTRYPOINT ["/tini", "--"]
CMD /opt/client.sh
