FROM #DOCKER_IMAGE 

ARG ARTIFACTORY_USERNAME
ARG ARTIFACTORY_PASSWORD

COPY docker-deps/ /keys

RUN cat /keys/artifactory_key.pub | apt-key add - && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    echo "deb https://${ARTIFACTORY_USERNAME}:${ARTIFACTORY_PASSWORD}@sixriver.jfrog.io/sixriver/debian xenial main" >> /etc/apt/sources.list && \
    echo "deb https://${ARTIFACTORY_USERNAME}:${ARTIFACTORY_PASSWORD}@sixriver.jfrog.io/sixriver/ros-ubuntu xenial main" >> /etc/apt/sources.list

RUN sh -c 'echo "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial main" >> /etc/apt/sources.list' && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ccache \
        libffi-dev \
        ruby \
        ruby-dev \
        rubygems \
        lsb-release \
        apt-transport-https \
        python-apt \
        apt-utils \
        wget \
        clang-6.0 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-6.0 1000 && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-6.0 1000 && \
    clang --version && \
    ln -s ../../bin/ccache /usr/lib/ccache/clang && \
    ln -s ../../bin/ccache /usr/lib/ccache/clang++ && \    
    gem install --no-ri --no-rdoc fpm && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN gem install fpm
