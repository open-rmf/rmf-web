FROM rmf-web/builder
COPY . /root/rmf-web
SHELL ["bash", "-c"]

RUN . /opt/rmf/setup.bash && npm config set unsafe-perm && cd /root/rmf-web && \
  CI=1 npm install -g lerna@4 && lerna bootstrap --scope=rmf-dashboard

RUN cd /root/rmf-web/packages/dashboard && \
  PUBLIC_URL='/dashboard' \
  REACT_APP_TRAJECTORY_SERVER='ws://localhost:8006' \
  REACT_APP_RMF_SERVER='https://example.com/rmf/api/v1' \
  REACT_APP_AUTH_PROVIDER='keycloak' \
  REACT_APP_KEYCLOAK_CONFIG='{ "realm": "rmf-web", "clientId": "dashboard", "url": "https://example.com/auth" }' \
  npm run build

###

FROM nginx:stable
COPY --from=0 /root/rmf-web/packages/dashboard/build /usr/share/nginx/html/dashboard
SHELL ["bash", "-c"]
RUN echo -e 'server {\n\
  listen 80;\n\
  server_name localhost;\n\
\n\
  location / {\n\
    root /usr/share/nginx/html;\n\
    index index.html index.htm;\n\
    try_files $uri /dashboard/index.html;\n\
  }\n\
\n\
  location /dashboard/static/ {\n\
    expires 30d;\n\
  }\n\
\n\
  error_page 500 502 503 504 /50x.html;\n\
  location = /50x.html {\n\
    root /usr/share/nginx/html;\n\
  }\n\
}\n' > /etc/nginx/conf.d/default.conf
