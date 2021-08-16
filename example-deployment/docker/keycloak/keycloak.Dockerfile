FROM quay.io/keycloak/keycloak:12.0.4
# You can find the json logger plugin here https://github.com/kill9zombie/keycloak_jsonlog_eventlistener
COPY ./jsonlog-event-listener.jar /opt/jboss/keycloak/standalone/deployments/jsonlog-event-listener.jar
