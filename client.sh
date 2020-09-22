#!/bin/bash

source /root/.bashrc
apt install -y build-essential pkg-config libssl-dev openssl swig
export LDFLAGS="-L/usr/lib"
export CFLAGS="-I/usr/include/openssl"
export CPPFLAGS="-I/usr/include/openssl"
export PKG_CONFIG_PATH="/usr/lib/pkgconfig"
/root/.pyenv/shims/pip install --upgrade pip
/root/.pyenv/shims/pip install -r /opt/rosws/src/converter/requirements.txt
# /root/.pyenv/shims/pip install PyYAML rospkg --no-cache-dir
# /root/.pyenv/shims/pip install python-qpid-proton --no-cache-dir
# /root/.pyenv/shims/pip install pillow --no-cache-dir

sleep infinity
