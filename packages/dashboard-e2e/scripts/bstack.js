// TODO: move this to a utils package.

const https = require('https');
const { mkdirSync, readdirSync, readFileSync, writeFileSync } = require('fs');
const { resolve } = require('path');

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

/**
 * Write browserstack metadata, this is meant to be called in the `beforeSuite` hook of `wdio.conf.js`.
 * @param {string} artifactsDir
 * @param {object} browser The `browser` object of the test.
 * @param {object} suite The `suite` object passed into the wdio hook.
 */
async function writeMetadata(artifactsDir, browser, suite) {
  const browserName = browser.requestedCapabilities.browserName;
  const bstackArtifact = {
    browserName,
    suite: suite.title,
    sessionId: browser.sessionId,
    publicUrl: await getPublicUrl(browser.sessionId),
  };
  const outDir = resolve(artifactsDir, 'browserstack');
  mkdirSync(outDir, { recursive: true });
  writeFileSync(
    `${outDir}/${suite.title}.${browserName}.json`,
    JSON.stringify(bstackArtifact, undefined, 2),
  );
}

/**
 * Annonate public urls into github workflows.
 * Should be called in the `onComplete` hook of a test run which called `writeMetadata`.
 * @param {string} artifactsDir Must be the same directory passed to `writeMetadata`.
 */
function annonatePublicUrls(artifactsDir) {
  try {
    const metadataDir = resolve(artifactsDir, 'browserstack');
    readdirSync(metadataDir).forEach((fileName) => {
      const contents = JSON.parse(readFileSync(resolve(metadataDir, fileName)));
      console.log(
        `::notice title=(${contents.browserName}) ${contents.suite}::${contents.publicUrl}`,
      );
    });
  } catch (e) {
    if (e.code === 'ENOENT') {
      console.log("directory doesn't exist, skipping browserstack annotations");
    } else {
      throw e;
    }
  }
}

module.exports = {
  getPublicUrl,
  writeMetadata,
  annonatePublicUrls,
};
