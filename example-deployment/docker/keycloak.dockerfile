FROM quay.io/keycloak/keycloak:11.0.0

COPY dp3-realm.json /

CMD [ \
  "-Dkeycloak.migration.action=import", \
  "-Dkeycloak.migration.provider=singleFile", \
  "-Dkeycloak.migration.file=/dp3-realm.json", \
  "-Dkeycloak.migration.realmName=dp3", \
  "-Dkeycloak.migration.strategy=IGNORE_EXISTING", \
  "-b", "0.0.0.0" \
]
