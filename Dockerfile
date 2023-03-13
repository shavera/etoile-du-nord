# syntax=docker/dockerfile:1.4
FROM shavera/cmake-base AS builder

# build edn
WORKDIR /tmp/edn-build
RUN --mount=type=bind,src=source/,dst=/tmp/edn <<EOT
    cmake /tmp/edn
    cmake --build .
EOT

CMD ["ctest"]
