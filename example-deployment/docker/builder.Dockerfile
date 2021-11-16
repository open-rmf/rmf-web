# base image used to build rmf-web packages
# NOTE: be sure to rebuild this when the depedencies change

FROM ubuntu:20.04

SHELL ["bash", "-c"]

RUN apt-get update && apt-get install -y curl && \
  curl -sL https://deb.nodesource.com/setup_14.x | bash - && \
  apt-get update && apt-get install -y nodejs python3 python3-pip

RUN pip3 install pipenv

COPY . /root/rmf-web
RUN cd /root/rmf-web && \
  npm install -g npm@latest && \
  npm config set unsafe-perm && \
  npm ci
