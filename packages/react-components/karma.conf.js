// FIXME: loading assets bundled by webpack results in 404, https://github.com/ryanclark/karma-webpack/issues/498

process.env.CHROME_BIN = require('puppeteer').executablePath();

process.env.TZ = 'Asia/Singapore';

const localIdentifier = `${Date.now().toString()}+${Math.random()}`;

module.exports = (config) => {
  const isCoverage = config.coverage ? true : false;
  const useBrowserStack = process.env.BROWSERSTACK_USERNAME && process.env.BROWSERSTACK_ACCESS_KEY;
  const testWebpackConfig = require('./webpack.config.js')({
    env: 'development',
    coverage: isCoverage,
  });

  config.set({
    client: {
      jasmine: {
        random: false,
        // Doesn't work due to https://github.com/karma-runner/karma-jasmine/issues/218
        // stopSpecOnExpectationFailure: true,
      },
    },

    browserStack: {
      username: process.env.BROWSERSTACK_USERNAME,
      accessKey: process.env.BROWSERSTACK_ACCESS_KEY,
      project: 'rmf-web',
      build: `react-components:${process.env.BROWSERSTACK_BUILD || 'local'}`,
      localIdentifier,
    },

    // base path that will be used to resolve all patterns (eg. files, exclude)
    basePath: '',

    // frameworks to use
    // available frameworks: https://npmjs.org/browse/keyword/karma-adapter
    frameworks: ['jasmine', 'source-map-support'],

    // list of files / patterns to load in the browser
    files: [
      {
        pattern: 'test/index.spec.js',
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
    // preprocessors: { 'lib/**/*spec.+(ts|tsx)': ['webpack'] },
    preprocessors: { 'test/index.spec.js': ['webpack'] },

    webpack: testWebpackConfig,

    webpackMiddleware: {
      // webpack-dev-middleware configuration
      // i. e.
      stats: 'errors-only',
    },

    coverageReporter: {
      dir: '.',
      subdir: '.',
      reporters: [{ type: 'text' }, { type: 'text-summary' }, { type: 'lcovonly' }],
    },

    // List of plugins to load. A plugin can be a string (in which case it will be required by
    // Karma or an inlined plugin - Object. By default, Karma loads all sibling NPM modules which
    // have a name starting with karma-*.
    plugins: [
      'karma-browserstack-launcher',
      'karma-chrome-launcher',
      'karma-coverage',
      'karma-jasmine',
      'karma-source-map-support',
      'karma-webpack',
    ],

    // test results reporter to use
    // possible values: 'dots', 'progress'
    // available reporters: https://npmjs.org/browse/keyword/karma-reporter
    reporters: [
      'dots',
      ...(isCoverage ? ['coverage'] : []),
      ...(useBrowserStack ? ['BrowserStack'] : []),
    ],

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
      bsSafari: {
        base: 'BrowserStack',
        browser: 'safari',
        browser_version: 'latest',
        os: 'OS X',
        os_version: 'Big Sur',
      },
    },
    captureTimeout: 210000,
    browserDisconnectTolerance: 3,
    browserDisconnectTimeout: 210000,
    browserNoActivityTimeout: 210000,

    // start these browsers
    // available browser launchers: https://npmjs.org/browse/keyword/karma-launcher
    browsers: [...(useBrowserStack ? ['bsSafari', 'bsChrome'] : ['ChromeHeadless'])],

    // Continuous Integration mode
    // if true, Karma captures browsers, runs the tests and exits
    singleRun: process.env.CI ? true : false,

    // Concurrency level
    // how many browser should be started simultaneous
    concurrency: Infinity,
  });
};
