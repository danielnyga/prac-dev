FROM ubuntu:14.04

# enable the universe
RUN sed -i 's/^#\s*\(deb.*universe\)$/\1/g' /etc/apt/sources.list
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y -q bash build-essential gfortran libatlas-base-dev \
    libffi-dev libssl-dev git python2.7 python2.7-dev python-pip \
    python-setuptools python-tk python-scipy python-apt \
    dvipng texlive-full default-jre libjpeg-dev libpng-dev pkg-config \
    libfreetype6-dev zlib1g-dev libxml2-dev libxslt1-dev g++

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
ADD requirements.txt /requirements.txt
RUN sudo pip install --upgrade pip && sudo pip install -U -r /requirements.txt

#RUN update-java-alternatives --jre -s java-1.7.0-openjdk-amd64
ENV JAVA_HOME /usr/lib/jvm/java-7-openjdk-amd64/jre
ENV PATH $JAVA_HOME/bin:$PATH

RUN mkdir pracmln
ADD src/pracmln pracmln
RUN mkdir prac
ADD src/prac prac

RUN cd pracmln && python make_apps.py --webmln --docs && chmod +x env.sh \
&& . ./env.sh && echo PYTHONPATH: && echo $PYTHONPATH && python test.py \
&& cd ../prac && python make_apps.py --pracweb && chmod +x env.sh \
&& . ./env.sh && echo PYTHONPATH: && echo $PYTHONPATH && python test.py

CMD ["/bin/bash"]