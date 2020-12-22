'use strict';

const path = require('path');

// Helper functions
const ROOT = path.resolve(__dirname, '..');

function root(args) {
  args = Array.prototype.slice.call(arguments, 0);
  return path.join.apply(path, [ROOT].concat(args));
}

/**
 * Webpack Constants
 */
const ENV = (process.env.ENV = process.env.NODE_ENV = 'development');
/**
 * Webpack configuration
 *
 * See: http://webpack.github.io/docs/configuration.html#cli
 */
module.exports = (options) => {
  return {
    entry: root() + '/lib/index.ts',
    // output: 'bundle.js',
    output: {
      filename: 'bundle.js',
    },
    mode: ENV,

    /**
     * Source map for Karma from the help of karma-sourcemap-loader &  karma-webpack
     */
    devtool: 'inline-source-map',

    /**
     * Options affecting the resolving of modules.
     * See: http://webpack.github.io/docs/configuration.html#resolve
     */
    resolve: {
      /**
       * An array of extensions that should be used to resolve modules.
       * See: http://webpack.github.io/docs/configuration.html#resolve-extensions
       */
      extensions: ['.ts', '.tsx', '.js', '.jsx', '.json'],

      /**
       * Make sure root is src
       */
      modules: [path.resolve(__dirname, 'lib'), 'node_modules'],
    },

    /**
     * Options affecting the normal modules.
     * See: http://webpack.github.io/docs/configuration.html#module
     */
    module: {
      rules: [
        /**
         * Source map loader support for *.js files
         * Extracts SourceMaps for source files that as added as sourceMappingURL comment.
         *
         * See: https://github.com/webpack/source-map-loader
         */
        {
          enforce: 'pre',
          test: /\.(tsx|ts)$/,
          loader: 'source-map-loader',
          exclude: [
            // these packages have problems with their sourcemaps
            root('node_modules/rxjs'),
          ],
        },

        /**
         * Typescript loader support for .ts
         * See: https://github.com/s-panferov/awesome-typescript-loader
         */
        {
          test: /\.(tsx|ts)$/,
          loader: 'awesome-typescript-loader',
          query: {
            // use inline sourcemaps for "karma-remap-coverage" reporter
            sourceMap: false,
            inlineSourceMap: true,
            compilerOptions: {
              removeComments: true,
            },
          },
          exclude: [/\.e2e\.ts$/],
        },
        {
          test: /\.(woff|woff2|eot|ttf|otf|svg)$/,
          use: [
            {
              loader: 'file-loader',
            },
          ],
        },
        {
          test: /\.css$/,
          use: ['style-loader', 'css-loader'],
        },

        {
          enforce: 'post',
          test: /\.(js|tsx)$/,
          loader: 'istanbul-instrumenter-loader',
          include: root('lib'),
          exclude: [/\.(e2e|spec|d|stories)\.ts$/, /node_modules/],
        },
      ],
    },

    /**
     * Include polyfills or mocks for various node stuff
     * Description: Node configuration
     *
     * See: https://webpack.github.io/docs/configuration.html#node
     */
    node: {
      global: true,
      crypto: 'empty',
      module: false,
      clearImmediate: false,
      setImmediate: false,
    },

    externals: {
      jsdom: 'window',
      cheerio: 'window',
      'react/lib/ExecutionEnvironment': 'true',
      'react/addons': 'true',
      'react/lib/ReactContext': 'window',
    },
  };
};
