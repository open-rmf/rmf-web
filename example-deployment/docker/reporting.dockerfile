FROM ubuntu:latest

RUN apt-get update && apt-get install -y curl && \
  curl -sL https://deb.nodesource.com/setup_12.x | bash - && \
  apt-get update && apt-get install -y nodejs python3-pip

RUN pip3 install pipenv

COPY . /root/rmf-web
SHELL ["bash", "-c"]

RUN . /opt/rmf/setup.bash && npm config set unsafe-perm && cd /root/rmf-web && \
  CI=1 npm run bootstrap -- packages/reporting

RUN cd /root/rmf-web/packages/dashboard && \
  PUBLIC_URL='/reporting' \
  REACT_APP_TRAJECTORY_SERVER='ws://localhost:8006' \
  REACT_APP_ROS2_BRIDGE_SERVER='wss://example.com/ros2' \
  REACT_APP_AUTH_PROVIDER='keycloak' \
  REACT_APP_KEYCLOAK_CONFIG='{ "realm": "rmf-web", "clientId": "dashboard", "url": "https://example.com/auth" }' \
  npm run build

###

FROM nginx:stable
COPY --from=0 /root/rmf-web/packages/reporting/build /usr/share/nginx/html/reporting
SHELL ["bash", "-c"]
RUN echo -e 'server {\n\
  location / {\n\
  root /usr/share/nginx/html;\n\
  index index.html index.htm;\n\
  try_files $uri /reporting/index.html;\n\
  }\n\
  }\n' > /etc/nginx/conf.d/reporting.conf
