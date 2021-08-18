ARG BUILDER_TAG
FROM rmf-web/builder:$BUILDER_TAG

COPY . /root/rmf-web
RUN cd /root/rmf-web && \
  lerna run prepare --include-dependencies --scope=api-server && \
  cd packages/api-server && \
  npm run prepack
