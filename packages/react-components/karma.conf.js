process.env.CHROME_BIN = require('puppeteer').executablePath();

module.exports = (config) => {
  const isCoverage = config.coverage ? true : false;
  const useBrowserStack = process.env.BROWSERSTACK_USERNAME && process.env.BROWSERSTACK_ACCESS_KEY;
  const testWebpackConfig = require('./webpack.config.js')({
    env: 'development',
    coverage: isCoverage,
  });

  config.set({
    browserStack: {
      username: process.env.BROWSERSTACK_USERNAME,
      accessKey: process.env.BROWSERSTACK_ACCESS_KEY,
    },

    // base path that will be used to resolve all patterns (eg. files, exclude)
    basePath: '',

    // frameworks to use
    // available frameworks: https://npmjs.org/browse/keyword/karma-adapter
    frameworks: ['jasmine'],

    // list of files / patterns to load in the browser
    files: [
      {
        pattern: 'lib/**/*spec.+(ts|tsx)',
        watched: false,
      },
      {
        pattern: 'test-data/assets/**/*',
        watched: false,
        included: false,
        served: true,
        nocache: false,
      },
    ],

    // list of files / patterns to exclude
    // exclude: [],

    // preprocess matching files before serving them to the browser
    // available preprocessors: https://npmjs.org/browse/keyword/karma-preprocessor
    preprocessors: { 'lib/**/*spec.+(ts|tsx)': ['webpack'] },

    webpack: testWebpackConfig,

    // Webpack please don't spam the console when running in karma!
    webpackMiddleware: {
      quiet: true,
      stats: {
        colors: true,
      },
    },

    coverageReporter: {
      dir: '.',
      subdir: '.',
      reporters: [{ type: 'text' }, { type: 'text-summary' }, { type: 'lcovonly' }],
    },

    // List of plugins to load. A plugin can be a string (in which case it will be required by
    // Karma or an inlined plugin - Object. By default, Karma loads all sibling NPM modules which
    // have a name starting with karma-*.
    // plugins: [],

    // test results reporter to use
    // possible values: 'dots', 'progress'
    // available reporters: https://npmjs.org/browse/keyword/karma-reporter
    reporters: ['dots', isCoverage && 'coverage', useBrowserStack && 'BrowserStack'].filter(
      (x) => x,
    ),

    port: 9876,

    // enable / disable colors in the output (reporters and logs)
    colors: true,

    // level of logging. Possible values:
    // config.LOG_DISABLE || config.LOG_ERROR || config.LOG_WARN || config.LOG_INFO || config.LOG_DEBUG
    logLevel: config.LOG_INFO,

    // enable / disable watching file and executing tests whenever any file changes
    autoWatch: true,

    customLaunchers: {
      bsChrome: {
        base: 'BrowserStack',
        browser: 'chrome',
        browser_version: 'latest',
        os: 'Windows',
        os_version: '10',
      },
    },

    // start these browsers
    // available browser launchers: https://npmjs.org/browse/keyword/karma-launcher
    browsers: [useBrowserStack && 'bsChrome', !useBrowserStack && 'ChromeHeadless'].filter(
      (x) => x,
    ),

    // Continuous Integration mode
    // if true, Karma captures browsers, runs the tests and exits
    singleRun: process.env.CI ? true : false,

    // Concurrency level
    // how many browser should be started simultaneous
    concurrency: Infinity,
  });
};
