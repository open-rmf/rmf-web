const https = require('https');

/**
 * Get the public url of a browserstack session.
 *
 * Authentication is gotten from BROWSERSTACK_USERNAME and BROWSERSTACK_ACCESS_KEY environment variables.
 */
async function getPublicUrl(sessionId) {
  return new Promise((res) => {
    let respBuf = Buffer.alloc(0);
    https
      .request(
        `https://api.browserstack.com/automate/sessions/${sessionId}.json`,
        {
          auth: `${process.env.BROWSERSTACK_USERNAME}:${process.env.BROWSERSTACK_ACCESS_KEY}`,
        },
        (resp) => {
          resp.on('data', (buf) => (respBuf = Buffer.concat([respBuf, buf])));
          resp.on('end', () => {
            const json = respBuf.toString();
            const sessionDetails = JSON.parse(json);
            res(sessionDetails.automation_session.public_url);
          });
        },
      )
      .end();
  });
}

module.exports = getPublicUrl;
