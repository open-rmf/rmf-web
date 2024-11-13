FROM ubuntu:24.04
ARG BRANCH=main

### Build dashboard

RUN apt update && apt install -y curl \
  # required for api-server, which is a dev dependency of rmf-dashboard-framework
  python3 python3-venv
RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
  # shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"
RUN pnpm env use --global lts

# fetch sources
RUN mkdir -p /ws \
  && curl -L https://github.com/open-rmf/rmf-web/archive/$BRANCH.tar.gz -o rmf_web.tar.gz \
  && tar zxf rmf_web.tar.gz -C /ws --strip-components=1

# install deps
RUN cd /ws \
  && pnpm install --filter rmf-dashboard-framework...

# replace configs and build
RUN cd /ws/packages/rmf-dashboard-framework \
  && pnpm build:example examples/demo

### Set up bare minimum dashboard image

FROM docker.io/ubuntu:24.04
COPY --from=0 /ws/packages/rmf-dashboard-framework/examples/demo/dist /opt/dashboard

RUN apt update && apt install -y curl

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"
RUN pnpm env use --global lts

RUN pnpm add --global serve

ENTRYPOINT ["serve", "-sn", "/opt/dashboard"]
