ARG BUILDER_TAG
FROM rmf-web/builder:$BUILDER_TAG

COPY . /root/rmf-web
RUN cd /root/rmf-web && \
  lerna run prepare --include-dependencies --scope=reporting

ARG PUBLIC_URL
ARG REACT_APP_REPORTING_SERVER
ARG REACT_APP_AUTH_PROVIDER
ARG REACT_APP_KEYCLOAK_CONFIG

RUN cd /root/rmf-web/packages/reporting && \
  PUBLIC_URL='/reporting' \
  REACT_APP_REPORTING_SERVER='https://example.com/logserver/api/v1' \
  REACT_APP_AUTH_PROVIDER='keycloak' \
  REACT_APP_KEYCLOAK_CONFIG='{ "realm": "rmf-web", "clientId": "reporting", "url": "https://example.com/auth" }' \
  npm run build

###

FROM nginx:stable
COPY --from=0 /root/rmf-web/packages/reporting/build /usr/share/nginx/html/reporting
SHELL ["bash", "-c"]
RUN echo -e 'server {\n\
  listen 80;\n\
  server_name localhost;\n\
\n\
  location / {\n\
    root /usr/share/nginx/html;\n\
    index index.html index.htm;\n\
    try_files $uri /reporting/index.html;\n\
  }\n\
\n\
  location /reporting/static/ {\n\
    expires 30d;\n\
  }\n\
\n\
  error_page 500 502 503 504 /50x.html;\n\
  location = /50x.html {\n\
    root /usr/share/nginx/html;\n\
  }\n\
}\n' > /etc/nginx/conf.d/default.conf
