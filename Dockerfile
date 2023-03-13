# syntax=docker/dockerfile:1.4
FROM shavera/cmake-base AS builder

# build edn
WORKDIR /tmp/edn-build
# Heredocs not working on github actions as of 13 March 2023
RUN --mount=type=bind,src=source/,dst=/tmp/edn <<EOT
    cmake /tmp/edn
    cmake --build .
EOT

CMD ["ctest"]
