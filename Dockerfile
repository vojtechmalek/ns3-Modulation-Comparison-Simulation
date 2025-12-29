
# ============================================================================
# BASE IMAGE CONFIGURATION
# ============================================================================

# Base image: Ubuntu 24.04 LTS
FROM ubuntu:24.04

# Prevent interactive prompts during package installation
ARG DEBIAN_FRONTEND=noninteractive

# NS-3 version to build
ARG NS3_VERSION=ns-3.46.1

# ============================================================================
# SYSTEM DEPENDENCIES
# ============================================================================

# Install core build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ \
    cmake \
    ninja-build \
    git \
    ca-certificates \
    python3 \
    python3-dev \
    pkg-config \
    ccache \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# NS-3 SETUP
# ============================================================================

# Set working directory for NS-3 installation
WORKDIR /opt

# Clone and build NS-3
RUN git clone https://gitlab.com/nsnam/ns-3-dev.git && \
    cd ns-3-dev && \
    echo "Using NS-3 version: ${NS3_VERSION}" && \
    git checkout ${NS3_VERSION} && \
    ./ns3 configure --enable-modules applications,spectrum,propagation,buildings,mobility && \
    ./ns3 build -j $(nproc)

# ============================================================================
# WORKSPACE SETUP
# ============================================================================

# Add NS-3 to PATH
ENV PATH="/opt/ns-3-dev:${PATH}"

# Set working directory for user files
WORKDIR /workspace

CMD ["/bin/bash"]