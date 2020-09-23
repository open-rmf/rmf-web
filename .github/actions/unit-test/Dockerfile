FROM docker.pkg.github.com/osrf/romi-dashboard/e2e

ENV CI=true

CMD npm ci --unsafe-perm \
  && npm run build \
  && npm run test:coverage
