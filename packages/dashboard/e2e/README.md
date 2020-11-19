# E2E workflow

This is to document the flow and interaction of the e2e services in the github environment when:

- starting up the services
- running the tests

## Starting up the services

Below is a diagram representing the flow of commands when running `npm run test:e2e`

![Flow of commands diagram](documentation-utils/e2e-E2e-start-process.png)

## Container and network interactions

Below is a diagram of how the containers and networks interact when up and running

![Containers and network interaction diagram](documentation-utils/e2e-Container-networks.png)

Broadly speaking, there are 3 major parts of the system:

- Romidashboard container. It holds the API server and the React dashboard
- Auth container
- auth_network, a known network where the Auth container is reachable 

Requests from the romidashboard to the auth container would be routed to the gateway ip address of `auth_network`. Docker would then perform a NAT to route the requests to the auth container and vice versa.