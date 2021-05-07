FROM quay.io/keycloak/keycloak:12.0.4

# COPY ./jsonlog-event-listener.jar /opt/jboss/keycloak/standalone/deployments/jsonlog-event-listener.jar

COPY ${PWD}/packages/keycloak/startup-script/ /opt/jboss/startup-scripts/