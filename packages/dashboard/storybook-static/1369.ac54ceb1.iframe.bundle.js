(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [1369, 69107],
  {
    '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js':
      (module) => {
        (module.exports = function _interopRequireDefault(obj) {
          return obj && obj.__esModule ? obj : { default: obj };
        }),
          (module.exports.__esModule = !0),
          (module.exports.default = module.exports);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildLocalizeFn/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = function buildLocalizeFn(args) {
            return function (dirtyIndex, options) {
              var valuesArray;
              if (
                'formatting' ===
                  (null != options && options.context ? String(options.context) : 'standalone') &&
                args.formattingValues
              ) {
                var defaultWidth = args.defaultFormattingWidth || args.defaultWidth,
                  width = null != options && options.width ? String(options.width) : defaultWidth;
                valuesArray = args.formattingValues[width] || args.formattingValues[defaultWidth];
              } else {
                var _defaultWidth = args.defaultWidth,
                  _width =
                    null != options && options.width ? String(options.width) : args.defaultWidth;
                valuesArray = args.values[_width] || args.values[_defaultWidth];
              }
              return valuesArray[
                args.argumentCallback ? args.argumentCallback(dirtyIndex) : dirtyIndex
              ];
            };
          }),
          (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/localize/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildLocalizeFn/index.js',
            ),
          ),
          _default = {
            ordinalNumber: function ordinalNumber(dirtyNumber, _options) {
              return Number(dirtyNumber) + '.';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['pr. n. št.', 'po n. št.'],
                abbreviated: ['pr. n. št.', 'po n. št.'],
                wide: ['pred našim štetjem', 'po našem štetju'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1. čet.', '2. čet.', '3. čet.', '4. čet.'],
                wide: ['1. četrtletje', '2. četrtletje', '3. četrtletje', '4. četrtletje'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['j', 'f', 'm', 'a', 'm', 'j', 'j', 'a', 's', 'o', 'n', 'd'],
                abbreviated: [
                  'jan.',
                  'feb.',
                  'mar.',
                  'apr.',
                  'maj',
                  'jun.',
                  'jul.',
                  'avg.',
                  'sep.',
                  'okt.',
                  'nov.',
                  'dec.',
                ],
                wide: [
                  'januar',
                  'februar',
                  'marec',
                  'april',
                  'maj',
                  'junij',
                  'julij',
                  'avgust',
                  'september',
                  'oktober',
                  'november',
                  'december',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['n', 'p', 't', 's', 'č', 'p', 's'],
                short: ['ned.', 'pon.', 'tor.', 'sre.', 'čet.', 'pet.', 'sob.'],
                abbreviated: ['ned.', 'pon.', 'tor.', 'sre.', 'čet.', 'pet.', 'sob.'],
                wide: ['nedelja', 'ponedeljek', 'torek', 'sreda', 'četrtek', 'petek', 'sobota'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'd',
                  pm: 'p',
                  midnight: '24.00',
                  noon: '12.00',
                  morning: 'j',
                  afternoon: 'p',
                  evening: 'v',
                  night: 'n',
                },
                abbreviated: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'poln.',
                  noon: 'pold.',
                  morning: 'jut.',
                  afternoon: 'pop.',
                  evening: 'več.',
                  night: 'noč',
                },
                wide: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'polnoč',
                  noon: 'poldne',
                  morning: 'jutro',
                  afternoon: 'popoldne',
                  evening: 'večer',
                  night: 'noč',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'd',
                  pm: 'p',
                  midnight: '24.00',
                  noon: '12.00',
                  morning: 'zj',
                  afternoon: 'p',
                  evening: 'zv',
                  night: 'po',
                },
                abbreviated: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'opoln.',
                  noon: 'opold.',
                  morning: 'zjut.',
                  afternoon: 'pop.',
                  evening: 'zveč.',
                  night: 'ponoči',
                },
                wide: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'opolnoči',
                  noon: 'opoldne',
                  morning: 'zjutraj',
                  afternoon: 'popoldan',
                  evening: 'zvečer',
                  night: 'ponoči',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
