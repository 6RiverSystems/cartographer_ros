FROM #DOCKER_IMAGE 

RUN apt-get update 
RUN apt-get install -y \
      build-essential \	
      ruby-dev \
      rubygems \
      libffi-dev 

RUN gem install fpm
