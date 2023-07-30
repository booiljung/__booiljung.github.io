# Dev Container for C++

`reinstall-cmake.sh`:

```sh
#!/usr/bin/env bash
#-------------------------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License. See https://go.microsoft.com/fwlink/?linkid=2090316 for license information.
#-------------------------------------------------------------------------------------------------------------
#
set -e

CMAKE_VERSION=${1:-"none"}

if [ "${CMAKE_VERSION}" = "none" ]; then
    echo "No CMake version specified, skipping CMake reinstallation"
    exit 0
fi

# Cleanup temporary directory and associated files when exiting the script.
cleanup() {
    EXIT_CODE=$?
    set +e
    if [[ -n "${TMP_DIR}" ]]; then
        echo "Executing cleanup of tmp files"
        rm -Rf "${TMP_DIR}"
    fi
    exit $EXIT_CODE
}
trap cleanup EXIT


echo "Installing CMake..."
apt-get -y purge --auto-remove cmake
mkdir -p /opt/cmake

architecture=$(dpkg --print-architecture)
case "${architecture}" in
    arm64)
        ARCH=aarch64 ;;
    amd64)
        ARCH=x86_64 ;;
    *)
        echo "Unsupported architecture ${architecture}."
        exit 1
        ;;
esac

CMAKE_BINARY_NAME="cmake-${CMAKE_VERSION}-linux-${ARCH}.sh"
CMAKE_CHECKSUM_NAME="cmake-${CMAKE_VERSION}-SHA-256.txt"
TMP_DIR=$(mktemp -d -t cmake-XXXXXXXXXX)

echo "${TMP_DIR}"
cd "${TMP_DIR}"

curl -sSL "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_BINARY_NAME}" -O
curl -sSL "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_CHECKSUM_NAME}" -O

sha256sum -c --ignore-missing "${CMAKE_CHECKSUM_NAME}"
sh "${TMP_DIR}/${CMAKE_BINARY_NAME}" --prefix=/opt/cmake --skip-license

ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake
ln -s /opt/cmake/bin/ctest /usr/local/bin/ctest
```

`Dockerfile`:

```dockerfile
FROM mcr.microsoft.com/devcontainers/cpp:0-ubuntu-20.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.22.2"

# Optionally install the cmake for vcpkg
COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

USER root

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update --no-install-recommends \
    && apt upgrade --no-install-recommends -y \
    && apt-get install --no-install-recommends -y \
    libsdl2-dev \
    libsdl2-dev \
    libboost-all-dev

WORKDIR /Downloads
RUN wget https://github.com/mavlink/MAVSDK/releases/download/v1.4.13/libmavsdk-dev_1.4.13_ubuntu20.04_amd64.deb
RUN apt install --no-install-recommends -y ./libmavsdk-dev_1.4.13_ubuntu20.04_amd64.deb
RUN rm ./libmavsdk-dev_1.4.13_ubuntu20.04_amd64.deb

EXPOSE 14550 14550/udp
EXPOSE 14560 14560/udp
EXPOSE 8080 8080/tcp

USER vscode

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>
```

`docker-compose.yml`:

```yml
version: '3'

services:
  focal-cpp:
    container_name: focal-cpp
    image: devcontainers/cpp:0-ubuntu-20.04
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - 14550:14550/udp
      - 14560:14560/udp
      - 8080:8080/tcp
    volumes:
      - ../..:/workspaces:cached
      - /var/run/docker.sock:/var/run/docker-host.sock
    command: /bin/bash -c "while sleep 1000; do :; done"
```

`devcontainer.json`:

```json
// For format details, see https://aka.ms/devcontainer.json. For config options, see the
// README at: https://github.com/devcontainers/templates/tree/main/src/cpp
{
	"name": "C++",
	"build": {
		"dockerfile": "Dockerfile"
	},
	"customizations": {
		"vscode": {
			"extensions": [
				"ms-azuretools.vscode-docker",
				"MS-CEINTL.vscode-language-pack-ko",
				"ms-vscode.cpptools-extension-pack"
			]
		}
	}

	// Features to add to the dev container. More info: https://containers.dev/features.
	// "features": {},

	// Use 'forwardPorts' to make a list of ports inside the container available locally.
	,"appPort": [ "14550:14550", "14550:14550/udp", "14560:14560", "14560:14560/udp", "8080:8080" ]

	// Use 'postCreateCommand' to run commands after the container is created.
	// "postCreateCommand": "gcc -v",

	// Configure tool-specific properties.
	// "customizations": {},

	// Uncomment to connect as root instead. More info: https://aka.ms/dev-containers-non-root.
	// "remoteUser": "root"
}
```

