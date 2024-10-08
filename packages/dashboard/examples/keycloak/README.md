This is an example using a local keycloak authentication provider.

Follow the instructions below to run the example

1. Start a keycloak instance

```bash
docker run --rm -p 8080:8080 -e KEYCLOAK_ADMIN=admin -e KEYCLOAK_ADMIN_PASSWORD=admin quay.io/keycloak/keycloak:25.0.2 start-dev
```

2. Setup keycloak for rmf-web

```bash
examples/keycloak/keycloak-setup.bash -u admin -o keycloak-example.pub
```

when prompted for the admin password, use "admin"

the script will

1. create a "rmf-web" realm
1. create a user in "rmf-web" realm with the same credentials as the keycloak admin
1. create a "dashboard" client
1. create a rmf api server client scope
1. create mappers for rmf api server client scopes
1. link the dashboard client with the client scope
1. export the public key

3. Start rmf api server with the keycloak config

```bash
# be sure to run from packages/dashboard directory and source a supported rmf installation
RMF_API_SERVER_CONFIG="$(pwd)/examples/keycloak/api_server_config.py" pnpm -C ../api-server start
```
