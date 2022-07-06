// public/env.js
window.ENV = {
  REACT_APP_TRAJECTORY_SERVER: 'ws://localhost:8006',
  // REACT_APP_AUTH_PROVIDER: "keycloak",
  // REACT_APP_KEYCLOAK_CONFIG: '{"realm": "rmf-web", "clientId": "dashboard", "url" : "https://localhost:8000/auth"}',
  REACT_APP_RMF_SERVER: 'http://localhost:8000',
  PUBLIC_URL: '/dashboard',
};

// ARG DOMAIN_URL="rmf-deployment-template.open-rmf.org"
// ENV PUBLIC_URL="/dashboard"
// ENV REACT_APP_TRAJECTORY_SERVER="wss://${DOMAIN_URL}/trajectory"
// ENV REACT_APP_RMF_SERVER="https://${DOMAIN_URL}/rmf/api/v1"
// ENV REACT_APP_AUTH_PROVIDER="keycloak"
// ENV REACT_APP_KEYCLOAK_CONFIG='{"realm": "rmf-web", "clientId": "dashboard", "url" : "https://'${DOMAIN_URL}'/auth"}'
