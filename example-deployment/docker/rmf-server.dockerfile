FROM rmf-web/builder

SHELL ["bash", "-c"]
COPY . /root/rmf-web
RUN . /opt/rmf/setup.bash && \
  npm config set unsafe-perm && \
  cd /root/rmf-web && \
  CI=1 npm run bootstrap -- packages/api-server

# cleanup
RUN rm -rf /var/lib/apt/lists && \
  npm cache clean --force

RUN echo -e '#!/bin/bash\n\
  . /opt/rmf/setup.bash\n\
  exec rmf_api_server "$@"\n\
  ' > /docker-entry-point.sh && chmod +x /docker-entry-point.sh

ENTRYPOINT ["/docker-entry-point.sh"]
CMD ["--help"]
