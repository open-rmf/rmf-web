const { baseUrl, getToken, request, tryRequest } = require('./utils');

function authHeaders(token) {
  return { Authorization: `bearer ${token}` };
}

(async () => {
  try {
    const token = await getToken();
    let resp;

    // creates the rmf-web realm.
    await tryRequest(
      `${baseUrl}/admin/realms/`,
      {
        method: 'POST',
        headers: authHeaders(token),
      },
      {
        realm: 'rmf-web',
        enabled: true,
      },
    );

    // creates the dashboard client
    await tryRequest(
      `${baseUrl}/admin/realms/rmf-web/clients`,
      {
        method: 'POST',
        headers: authHeaders(token),
      },
      {
        clientId: 'dashboard',
        rootUrl: 'https://openrobotics.demo.open-rmf.org',
        redirectUris: ['https://openrobotics.demo.open-rmf.org/*'],
        webOrigins: ['https://openrobotics.demo.open-rmf.org'],
        publicClient: true,
      },
    );

    await tryRequest(
      `${baseUrl}/admin/realms/rmf-web/clients`,
      {
        method: 'POST',
        headers: authHeaders(token),
      },
      {
        clientId: 'reporting',
        rootUrl: 'https://openrobotics.demo.open-rmf.org',
        redirectUris: ['https://openrobotics.demo.open-rmf.org/*'],
        webOrigins: ['https://openrobotics.demo.open-rmf.org'],
        publicClient: true,
      },
    );

    // create example user
    await tryRequest(
      `${baseUrl}/admin/realms/rmf-web/users`,
      {
        method: 'POST',
        headers: authHeaders(token),
      },
      {
        username: 'example',
        enabled: true,
      },
    );

    // get the newly created user
    resp = await request(`${baseUrl}/admin/realms/rmf-web/users?username=example`, {
      method: 'GET',
      headers: authHeaders(token),
    });
    const user = JSON.parse(resp.body)[0];

    // set the password
    await request(
      `${baseUrl}/admin/realms/rmf-web/users/${user.id}/reset-password`,
      {
        method: 'PUT',
        headers: authHeaders(token),
      },
      {
        value: 'example',
        temporary: false,
      },
    );

    await tryRequest(
      `${baseUrl}/admin/realms/rmf-web/events/config`,
      {
        method: 'PUT',
        headers: authHeaders(token),
      },
      {
        eventsEnabled: true,
        eventsListeners: ['jsonlog_event_listener'],
      },
    );

    // create audience client scope
    await request(
      `${baseUrl}/admin/realms/rmf-web/client-scopes`,
      {
        method: 'POST',
        headers: authHeaders(token),
      },
      {
        name: 'dashboard',
	protocol: 'openid-connect',
	description: 'dashboard scope',
	protocolMappers: [
	  {
	    config: {
	      "access.token.claim": "true",
	      "id.token.claim": "false",
	      "included.client.audience": "dashboard"
	    },
	    name: 'rmf-audience',
 	    protocol: 'openid-connect',
  	    protocolMapper: 'oidc-audience-mapper'
	  }
	]
      },
    );

    // get dashboard id ( not clientid )
    resp = await request(`${baseUrl}/admin/realms/rmf-web/clients`, {
      method: 'GET',
      headers: authHeaders(token),
    });

    const client_array = JSON.parse(resp.body);
    const dashboard_id = client_array.filter(function(item){
	    return item.clientId == "dashboard";
    })[0].id

    // get client scope id 
    resp = await request(`${baseUrl}/admin/realms/rmf-web/client-scopes`, {
      method: 'GET',
      headers: authHeaders(token),
    });

    const client_scopes_array = JSON.parse(resp.body);
    const client_scope_id = client_scopes_array.filter(function(item){
	    return item.name == "dashboard";
    })[0].id
    console.log(client_scope_id)

    // set client with client scope
    await request(
      `${baseUrl}/admin/realms/rmf-web/clients/${dashboard_id}/default-client-scopes/${client_scope_id}`,
      {
        method: 'PUT',
        headers: authHeaders(token),
      }
    );
   

  } catch (e) {
    process.exitCode = 1;
  }
})();


