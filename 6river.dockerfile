FROM ros:kinetic 

RUN apt-get update &&\
    apt-get install -y \
      build-essential \
      ruby-dev \
      rubygems
RUN gem install fpm
