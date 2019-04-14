FROM #DOCKER_IMAGE 

ARG ARTIFACTORY_USERNAME
ARG ARTIFACTORY_PASSWORD

COPY docker-deps/ /keys

RUN cat /keys/artifactory_key.pub | apt-key add - && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    echo "deb https://${ARTIFACTORY_USERNAME}:${ARTIFACTORY_PASSWORD}@sixriver.jfrog.io/sixriver/debian xenial main" >> /etc/apt/sources.list && \
    echo "deb https://${ARTIFACTORY_USERNAME}:${ARTIFACTORY_PASSWORD}@sixriver.jfrog.io/sixriver/ros-ubuntu xenial main" >> /etc/apt/sources.list

RUN apt-get update 
RUN apt-get install -y \
      build-essential \	
      ruby-dev \
      rubygems \
      libffi-dev 

RUN gem install fpm
