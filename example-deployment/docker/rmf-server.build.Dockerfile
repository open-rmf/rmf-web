ARG BUILDER_TAG
FROM rmf-web/builder:$BUILDER_TAG

COPY . /root/rmf-web
RUN cd /root/rmf-web && \
  cd packages/api-server && \
  npm run prepack
