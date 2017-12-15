FROM ros:kinetic 

RUN apt-get update &&\
    apt-get install -y \
      build-essential \
      libffi-devel \
      ruby-dev \
      rubygems \
      libxml2 \
      libxml2-dev \
      libxslt1-dev
RUN gem install fpm
