ARG BASE_IMAGE=docker.io/ubuntu:24.04
FROM $BASE_IMAGE
ARG BRANCH=main

### Build dashboard

# fetch sources
RUN mkdir -p /ws \
  && curl -L https://github.com/open-rmf/rmf-web/archive/$BRANCH.tar.gz -o rmf_web.tar.gz \
  && tar zxf rmf_web.tar.gz -C /ws --strip-components=1

# install deps
RUN cd /ws \
  && pnpm install --filter rmf-dashboard...

# build
RUN cd /ws/packages/dashboard \
  && pnpm run build

### Set up bare minimum dashboard image

FROM docker.io/ubuntu:24.04
COPY --from=0 /ws/packages/dashboard/dist /opt/dashboard

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"

# nodejs seems to have changed the official mirror, the default in pnpm is very slow now
RUN pnpm config -g set 'node-mirror:release' https://nodejs.org/dist && pnpm env use --global lts

RUN npm install --global serve

ENTRYPOINT serve -s /opt/dashboard/
