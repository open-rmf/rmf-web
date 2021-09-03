ARG BASE_IMAGE=ghcr.io/open-rmf/rmf/rmf_demos
FROM $BASE_IMAGE

RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | apt-key add - && \
  sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list' && \
  curl -sL https://deb.nodesource.com/setup_14.x | bash - && \
  apt-get update && apt-get install -y \
    nodejs \
    google-chrome-stable \
    jq && \
  pip3 install pipenv && \
  npm config set unsafe-perm
