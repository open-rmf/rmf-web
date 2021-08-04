FROM rmf-web/builder
COPY . /root/rmf-web
SHELL ["bash", "-c"]

RUN npm config set unsafe-perm && cd /root/rmf-web && \
  CI=1 npm install -g lerna@4 && lerna bootstrap --scope=minimal

RUN cd /root/rmf-web/packages/minimal && \
  PUBLIC_URL='/minimal' \
  REACT_APP_TRAJECTORY_SERVER='ws://localhost:8006' \
  REACT_APP_RMF_SERVER='https://example.com/rmf/api/v1' \
  REACT_APP_AUTH_PROVIDER='keycloak' \
  REACT_APP_KEYCLOAK_CONFIG='{ "realm": "rmf-web", "clientId": "minimal", "url": "https://example.com/auth" }' \
  npm run build

###

FROM nginx:stable
COPY --from=0 /root/rmf-web/packages/minimal/build /usr/share/nginx/html/minimal
SHELL ["bash", "-c"]
RUN echo -e 'server {\n\
  location / {\n\
  root /usr/share/nginx/html;\n\
  index index.html index.htm;\n\
  try_files $uri /minimal/index.html;\n\
  }\n\
  }\n' > /etc/nginx/conf.d/minimal.conf
