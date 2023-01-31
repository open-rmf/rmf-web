ARG BASE_IMAGE=ghcr.io/open-rmf/rmf/rmf_demos
FROM $BASE_IMAGE

RUN curl -sL https://deb.nodesource.com/setup_16.x | bash - && \
  apt-get update && apt-get install -y \
    nodejs \
    jq && \
  pip3 install pipenv && \
  npm config set unsafe-perm
