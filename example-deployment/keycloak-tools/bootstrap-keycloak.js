const { baseUrl, getToken, request } = require('./utils');

function authHeaders(token) {
  return { Authorization: `bearer ${token}` };
}

(async () => {
  const token = await getToken();

  // creates the rmf-web realm.
  await request(
    `${baseUrl}/admin/realms/`,
    {
      method: 'POST',
      headers: authHeaders(token),
    },
    {
      realm: 'rmf-web',
    },
  );

  // creates the dashboard client
  await request(
    `${baseUrl}/admin/realms/rmf-web/clients`,
    {
      method: 'POST',
      headers: authHeaders(token),
    },
    {
      clientId: 'dashboard',
    },
  );
})();
