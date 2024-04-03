(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [50049, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/localize/index.js':
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
                narrow: ['ie.', 'isz.'],
                abbreviated: ['i. e.', 'i. sz.'],
                wide: ['Krisztus előtt', 'időszámításunk szerint'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1.', '2.', '3.', '4.'],
                abbreviated: ['1. n.év', '2. n.év', '3. n.év', '4. n.év'],
                wide: ['1. negyedév', '2. negyedév', '3. negyedév', '4. negyedév'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
              formattingValues: {
                narrow: ['I.', 'II.', 'III.', 'IV.'],
                abbreviated: ['I. n.év', 'II. n.év', 'III. n.év', 'IV. n.év'],
                wide: ['I. negyedév', 'II. negyedév', 'III. negyedév', 'IV. negyedév'],
              },
              defaultFormattingWidth: 'wide',
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['J', 'F', 'M', 'Á', 'M', 'J', 'J', 'A', 'Sz', 'O', 'N', 'D'],
                abbreviated: [
                  'jan.',
                  'febr.',
                  'márc.',
                  'ápr.',
                  'máj.',
                  'jún.',
                  'júl.',
                  'aug.',
                  'szept.',
                  'okt.',
                  'nov.',
                  'dec.',
                ],
                wide: [
                  'január',
                  'február',
                  'március',
                  'április',
                  'május',
                  'június',
                  'július',
                  'augusztus',
                  'szeptember',
                  'október',
                  'november',
                  'december',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['V', 'H', 'K', 'Sz', 'Cs', 'P', 'Sz'],
                short: ['V', 'H', 'K', 'Sze', 'Cs', 'P', 'Szo'],
                abbreviated: ['V', 'H', 'K', 'Sze', 'Cs', 'P', 'Szo'],
                wide: ['vasárnap', 'hétfő', 'kedd', 'szerda', 'csütörtök', 'péntek', 'szombat'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'de.',
                  pm: 'du.',
                  midnight: 'éjfél',
                  noon: 'dél',
                  morning: 'reggel',
                  afternoon: 'du.',
                  evening: 'este',
                  night: 'éjjel',
                },
                abbreviated: {
                  am: 'de.',
                  pm: 'du.',
                  midnight: 'éjfél',
                  noon: 'dél',
                  morning: 'reggel',
                  afternoon: 'du.',
                  evening: 'este',
                  night: 'éjjel',
                },
                wide: {
                  am: 'de.',
                  pm: 'du.',
                  midnight: 'éjfél',
                  noon: 'dél',
                  morning: 'reggel',
                  afternoon: 'délután',
                  evening: 'este',
                  night: 'éjjel',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
