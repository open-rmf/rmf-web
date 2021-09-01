const https = require('https');
const { createWriteStream, mkdirSync } = require('fs');
const { resolve } = require('path');

const outDir = resolve(`${__dirname}/../artifacts`);

/**
 * Download videos from browserstack and save it to the "artifacts" directory.
 *
 * Authentication is gotten from BROWSERSTACK_USERNAME and BROWSERSTACK_ACCESS_KEY environment variables.
 * @param {object} sessionsJson an object with key as the session names and values the session ids.
 */
module.exports = async function downloadBstackVideos(sessions) {
  mkdirSync(outDir, { recursive: true });

  const downloadVideo = async (sessionName, sessionId) => {
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
              const videoUrl = sessionDetails.automation_session.video_url;
              https
                .request(videoUrl, (resp) => {
                  resp.pipe(createWriteStream(`${outDir}/${sessionName}.mp4`));
                  resp.on('end', res);
                })
                .end();
            });
          },
        )
        .end();
    });
  };

  for (const [sessionName, sessionId] of Object.entries(sessions)) {
    await downloadVideo(sessionName, sessionId);
  }
};
