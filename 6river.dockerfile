FROM #DOCKER_IMAGE 

RUN apt-get update &&\
    apt-get install -y \
      build-essential \
      ruby-dev \
      rubygems \
      libffi-dev \
      pcl
     
RUN gem install fpm
