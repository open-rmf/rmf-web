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
        rootUrl: 'https://example.com',
        redirectUris: ['https://example.com/*'],
        webOrigins: ['https://example.com'],
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
        rootUrl: 'https://example.com',
        redirectUris: ['https://example.com/*'],
        webOrigins: ['https://example.com'],
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
  } catch (e) {
    process.exitCode = 1;
  }
})();
