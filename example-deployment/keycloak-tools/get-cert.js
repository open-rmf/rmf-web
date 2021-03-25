const { request, baseUrl } = require('./utils');

(async () => {
  // get the jws
  let resp = await request(`${baseUrl}/realms/rmf-web/.well-known/openid-configuration`);
  const jwksUri = JSON.parse(resp.body).jwks_uri;
  resp = await request(jwksUri);
  const jwks = JSON.parse(resp.body);
  const jws = jwks.keys[0];
  if (!jws) {
    process.exitCode = 1;
    return;
  }
  const x5c = jws.x5c[0];
  const pem = `-----BEGIN CERTIFICATE-----\n${x5c}\n-----END CERTIFICATE-----`;

  console.log(pem);
})();
