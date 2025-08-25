#!/usr/bin/env bash
set -e

## PX4 Environment Setup Script - Multi-arch, Minimal/Full Install
## Supports Ubuntu 18.04 / 20.04 / 22.04 (x86_64, aarch64/ARM)
## Usage: ./setup_px4.sh [--no-nuttx] [--no-sim-tools]

INSTALL_NUTTX=true
INSTALL_SIM=true
INSTALL_ARCH=$(uname -m)

# Normalize ARM arch names
if [[ "$INSTALL_ARCH" =~ ^(armv7l|armhf)$ ]]; then
  INSTALL_ARCH="armhf"
elif [[ "$INSTALL_ARCH" =~ ^(aarch64|arm64)$ ]]; then
  INSTALL_ARCH="aarch64"
fi

# Parse script arguments
for arg in "$@"; do
  [[ $arg == "--no-nuttx" ]] && INSTALL_NUTTX=false
  [[ $arg == "--no-sim-tools" ]] && INSTALL_SIM=false
done

echo "PX4 setup on $INSTALL_ARCH"

# Docker environment check
if [ -f /.dockerenv ]; then
  echo "Running inside Docker, installing minimal dependencies."
  apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get -y install ca-certificates gnupg lsb-core sudo wget
fi

# Directory and requirements check
DIR=$( cd "$( dirname "${BASH_SOURCE}" )" && pwd )
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
  echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as setup_px4.sh (${DIR})."
  exit 1
fi

# Ubuntu version detection
UBUNTU_RELEASE="$(lsb_release -rs)"
case "${UBUNTU_RELEASE}" in
  "18.04"| "20.04"| "22.04") echo "Ubuntu ${UBUNTU_RELEASE}";;
  *) echo "Unsupported Ubuntu version: ${UBUNTU_RELEASE}"; exit 1;;
esac

echo "Installing PX4 general dependencies..."
sudo apt-get update -y --quiet
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
  astyle build-essential cmake cppcheck file g++ gcc gdb git lcov libfuse2 libxml2-dev \
  libxml2-utils make ninja-build python3 python3-dev python3-pip python3-setuptools \
  python3-wheel rsync shellcheck unzip zip

# Install ARM-native compilers for ARM hosts (no cross)
if [[ "$INSTALL_ARCH" == "aarch64" ]]; then
  sudo apt-get install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
elif [[ "$INSTALL_ARCH" == "armhf" ]]; then
  sudo apt-get install -y gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
fi

if [[ -n "$VIRTUAL_ENV" ]]; then
  python3 -m pip install -r ${DIR}/requirements.txt --upgrade-strategy only-if-needed
else
  python3 -m pip install --user -r ${DIR}/requirements.txt --upgrade-strategy only-if-needed
fi

# NuttX toolchain - ONLY for x86_64 hosts
if $INSTALL_NUTTX && [[ "$INSTALL_ARCH" == "x86_64" ]]; then
  echo "Installing NuttX dependencies and toolchain..."
  sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    automake binutils-dev bison build-essential flex gdb-multiarch genromfs gettext \
    gperf libelf-dev libexpat-dev libgmp-dev libisl-dev libmpc-dev libmpfr-dev libncurses5 \
    libncurses5-dev libncursesw5-dev libtool pkg-config screen texinfo u-boot-tools \
    util-linux vim-common
  if [[ "$UBUNTU_RELEASE" =~ ^(20.04|22.04)$ ]]; then
    sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install kconfig-frontends
  fi
  sudo usermod -aG dialout $USER
  NUTTX_GCC_VERSION="9-2020-q2-update"
  NUTTX_GCC_VERSION_SHORT="9-2020q2"
  source $HOME/.profile
  if which arm-none-eabi-gcc &>/dev/null; then
    GCC_VER_STR=$(arm-none-eabi-gcc --version)
    if grep -q "${NUTTX_GCC_VERSION}" <<< "$GCC_VER_STR"; then
      echo "arm-none-eabi-gcc-${NUTTX_GCC_VERSION} found, skipping installation"
    else
      echo "Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}"
      wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 \
        https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/${NUTTX_GCC_VERSION_SHORT}/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-x86_64-linux.tar.bz2
      sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/
      exportline="export PATH=/opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin:\$PATH"
      grep -Fxq "$exportline" $HOME/.profile || echo "$exportline" >> $HOME/.profile
      source $HOME/.profile
    fi
  fi
else
  echo "Skipping NuttX toolchain setup for non-x86_64 architecture: $INSTALL_ARCH"
fi

# Simulation tools - ONLY for x86_64 hosts
if $INSTALL_SIM && [[ "$INSTALL_ARCH" == "x86_64" ]]; then
  echo "Installing PX4 simulation tools..."
  sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install bc
  if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then java_version=11
  elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then java_version=13
  elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then java_version=11; fi
  sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    ant openjdk-$java_version-jre openjdk-$java_version-jdk libvecmath-java
  sudo update-alternatives --set java $(update-alternatives --list java | grep "java-$java_version")
  if [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
      sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update -y --quiet
    gazebo_packages="gz-garden"
  else
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1A1BF1B95C17494
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
    sudo apt-get update -y --quiet
    if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
      gazebo_packages="gazebo9 libgazebo9-dev"
    else
      gazebo_packages="gazebo11 libgazebo11-dev"
    fi
  fi
  sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    dmidecode $gazebo_packages gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    libeigen3-dev libgstreamer-plugins-base1.0-dev libimage-exiftool-perl \
    libopencv-dev libxml2-utils pkg-config protobuf-compiler
else
  echo "Skipping simulation tools for non-x86_64 architecture: $INSTALL_ARCH"
fi

echo "PX4 development environment setup done for $INSTALL_ARCH on Ubuntu $UBUNTU_RELEASE"
