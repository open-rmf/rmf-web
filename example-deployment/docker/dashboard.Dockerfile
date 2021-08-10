ARG BUILDER_TAG
FROM rmf-web/builder:$BUILDER_TAG

COPY . /root/rmf-web
RUN cd /root/rmf-web && \
  lerna run prepare --include-dependencies --scope=rmf-dashboard

ARG PUBLIC_URL
ARG REACT_APP_TRAJECTORY_SERVER
ARG REACT_APP_RMF_SERVER
ARG REACT_APP_AUTH_PROVIDER
ARG REACT_APP_KEYCLOAK_CONFIG

RUN cd /root/rmf-web/packages/dashboard && npm run build

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
  error_page 500 502 503 504 /50x.html;\n\
  location = /50x.html {\n\
    root /usr/share/nginx/html;\n\
  }\n\
}\n' > /etc/nginx/conf.d/default.conf
