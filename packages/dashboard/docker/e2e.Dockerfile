FROM docker.pkg.github.com/osrf/rmf-web/e2e/rmf:nightly

RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | apt-key add - \
  && sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list' \
  && curl -sL https://deb.nodesource.com/setup_12.x | bash - \
  && apt-get install -y nodejs google-chrome-stable docker.io docker-compose \
  && rm -rf /var/lib/apt/lists/*

ADD packages/dashboard/docker/chrome-no-sandbox /
