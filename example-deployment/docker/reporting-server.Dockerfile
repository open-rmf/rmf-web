ARG BUILDER_TAG
FROM rmf-web/builder:$BUILDER_TAG

COPY . /root/rmf-web
RUN cd /root/rmf-web && \
  lerna run prepare --include-dependencies --scope=reporting-server && \
  cd packages/reporting-server && \
  npm run prepack

FROM python:3.9

SHELL ["bash", "-c"]

COPY --from=0 /root/rmf-web/packages/reporting-server/dist/ /root/reporting-server

RUN cd /root/reporting-server && \
  pip3 install $(ls -1 | grep '.*.whl')[postgres]

RUN echo -e '#!/bin/bash\n\
  exec reporting_server "$@"\n\
  ' > /docker-entry-point.sh && chmod +x /docker-entry-point.sh
ENTRYPOINT ["/docker-entry-point.sh"]
