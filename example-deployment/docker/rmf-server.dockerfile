FROM rmf-web/builder

SHELL ["bash", "-c"]
COPY . /root/rmf-web
RUN . /opt/rmf/setup.bash && \
  npm config set unsafe-perm && \
  cd /root/rmf-web && \
  CI=1 npm install -g lerna@4 && lerna bootstrap --scope=api-server && \
  cd packages/api-server && \
  npm run prepack

FROM rmf-web/builder

COPY --from=0 /root/rmf-web/packages/api-server/dist/ .

SHELL ["bash", "-c"]
RUN . /opt/rmf/setup.bash && \
  pip3 install $(ls -1 | grep '.*.whl')[postgres]

# cleanup
RUN rm -rf /var/lib/apt/lists && \
  npm cache clean --force

RUN echo -e '#!/bin/bash\n\
  . /opt/rmf/setup.bash\n\
  exec rmf_api_server "$@"\n\
  ' > /docker-entry-point.sh && chmod +x /docker-entry-point.sh

ENTRYPOINT ["/docker-entry-point.sh"]
CMD ["--help"]
