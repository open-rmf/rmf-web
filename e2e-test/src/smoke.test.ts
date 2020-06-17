import { NightwatchBrowser } from 'nightwatch';

export = {
  'smoke test': function(browser: NightwatchBrowser) {
    browser
      .url('http://localhost:5000')
      .waitForElementVisible('#schedule-visualizer')
      .end();
  },
};
