ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE

FROM $BASE_IMAGE

### Build dashboard

RUN apt update && apt install -y curl \
  # required for api-server, which is a dev dependency of rmf-dashboard-framework
  python3 python3-venv
RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME=/root/.local/share/pnpm
ENV PATH="$PNPM_HOME/bin:$PATH"
RUN pnpm runtime set node lts -g

# fetch sources
RUN mkdir -p /ws \
  && BRANCH_NAME=$ROS_DISTRO && if [ "$ROS_DISTRO" = "rolling" ]; then BRANCH_NAME=main; fi \
  && curl -L https://github.com/open-rmf/rmf-web/archive/$BRANCH_NAME.tar.gz -o rmf_web.tar.gz \
  && tar zxf rmf_web.tar.gz -C /ws --strip-components=1

# install deps
# fallback to approve builds if pnpm install fails due to ignored scripts in non-interactive environment
RUN cd /ws \
  && (pnpm install --filter rmf-dashboard-framework... || (pnpm approve-builds --all && pnpm install --filter rmf-dashboard-framework...))

# replace configs and build
RUN cd /ws/packages/rmf-dashboard-framework \
  && pnpm build:example examples/demo

### Set up bare minimum dashboard image

FROM docker.io/ubuntu:24.04
COPY --from=0 /ws/packages/rmf-dashboard-framework/examples/demo/dist /opt/dashboard

# libatomic1 is required by pnpm binary on clean ubuntu images
RUN apt update && apt install -y curl libatomic1

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME=/root/.local/share/pnpm
ENV PATH="$PNPM_HOME/bin:$PATH"
RUN pnpm runtime set node lts -g

RUN pnpm add --global serve

ENTRYPOINT ["serve", "-sn", "/opt/dashboard"]
