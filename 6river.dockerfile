FROM #DOCKER_IMAGE 

RUN apt-get update &&\
    apt-get install -y \
      build-essential \
      ruby-dev \
      rubygems \
      libxml2 \
      libxml2-dev \
      libxslt1-dev
RUN gem install fpm
