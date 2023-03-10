# syntax=docker/dockerfile:1.4
FROM ubuntu:22.04 AS builder

# install build tooling and cmake from kitware repo
RUN <<EOT
  apt-get update
  apt-get install -y gpg wget build-essential ninja-build git
  wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
  echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
  apt-get update
  apt-get install -y cmake
EOT

# build edn
WORKDIR /tmp/edn-build
RUN --mount=type=bind,src=source/,dst=/tmp/edn <<EOT
    cmake /tmp/edn
    cmake --build .
EOT

CMD ["ctest"]
