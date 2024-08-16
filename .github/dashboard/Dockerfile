ARG BASE_IMAGE
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

# replace configs and build
COPY app-config.json /ws/packages/dashboard/app-config.json
RUN cd /ws/packages/dashboard \
  && pnpm run build

### Set up bare minimum dashboard image

FROM docker.io/ubuntu:24.04
COPY --from=0 /ws/packages/dashboard/dist /opt/dashboard

RUN apt update && apt install -y curl

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"

# nodejs seems to have changed the official mirror, the default in pnpm is very slow now
RUN pnpm config -g set 'node-mirror:release' https://nodejs.org/dist && pnpm env use --global lts

RUN npm install --global serve
ENV PATH "$(npm bin -g):$PATH"

COPY inject-env.sh /opt/inject-env.sh

ENTRYPOINT ["bash", "-c", ". /opt/inject-env.sh && npx serve -s /opt/dashboard"]
