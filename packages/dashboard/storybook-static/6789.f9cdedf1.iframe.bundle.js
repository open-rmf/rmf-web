(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [6789, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/localize/index.js':
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
          monthValues = {
            narrow: ['J', 'F', 'M', 'A', 'M', 'J', 'J', 'A', 'S', 'O', 'N', 'D'],
            abbreviated: [
              'Jan',
              'Feb',
              'Mär',
              'Apr',
              'Mai',
              'Jun',
              'Jul',
              'Aug',
              'Sep',
              'Okt',
              'Nov',
              'Dez',
            ],
            wide: [
              'Januar',
              'Februar',
              'März',
              'April',
              'Mai',
              'Juni',
              'Juli',
              'August',
              'September',
              'Oktober',
              'November',
              'Dezember',
            ],
          },
          formattingMonthValues = {
            narrow: monthValues.narrow,
            abbreviated: [
              'Jan.',
              'Feb.',
              'März',
              'Apr.',
              'Mai',
              'Juni',
              'Juli',
              'Aug.',
              'Sep.',
              'Okt.',
              'Nov.',
              'Dez.',
            ],
            wide: monthValues.wide,
          },
          _default = {
            ordinalNumber: function ordinalNumber(dirtyNumber) {
              return Number(dirtyNumber) + '.';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['v.Chr.', 'n.Chr.'],
                abbreviated: ['v.Chr.', 'n.Chr.'],
                wide: ['vor Christus', 'nach Christus'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1. Quartal', '2. Quartal', '3. Quartal', '4. Quartal'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: monthValues,
              formattingValues: formattingMonthValues,
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['S', 'M', 'D', 'M', 'D', 'F', 'S'],
                short: ['So', 'Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa'],
                abbreviated: ['So.', 'Mo.', 'Di.', 'Mi.', 'Do.', 'Fr.', 'Sa.'],
                wide: [
                  'Sonntag',
                  'Montag',
                  'Dienstag',
                  'Mittwoch',
                  'Donnerstag',
                  'Freitag',
                  'Samstag',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'vm.',
                  pm: 'nm.',
                  midnight: 'Mitternacht',
                  noon: 'Mittag',
                  morning: 'Morgen',
                  afternoon: 'Nachm.',
                  evening: 'Abend',
                  night: 'Nacht',
                },
                abbreviated: {
                  am: 'vorm.',
                  pm: 'nachm.',
                  midnight: 'Mitternacht',
                  noon: 'Mittag',
                  morning: 'Morgen',
                  afternoon: 'Nachmittag',
                  evening: 'Abend',
                  night: 'Nacht',
                },
                wide: {
                  am: 'vormittags',
                  pm: 'nachmittags',
                  midnight: 'Mitternacht',
                  noon: 'Mittag',
                  morning: 'Morgen',
                  afternoon: 'Nachmittag',
                  evening: 'Abend',
                  night: 'Nacht',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'vm.',
                  pm: 'nm.',
                  midnight: 'Mitternacht',
                  noon: 'Mittag',
                  morning: 'morgens',
                  afternoon: 'nachm.',
                  evening: 'abends',
                  night: 'nachts',
                },
                abbreviated: {
                  am: 'vorm.',
                  pm: 'nachm.',
                  midnight: 'Mitternacht',
                  noon: 'Mittag',
                  morning: 'morgens',
                  afternoon: 'nachmittags',
                  evening: 'abends',
                  night: 'nachts',
                },
                wide: {
                  am: 'vormittags',
                  pm: 'nachmittags',
                  midnight: 'Mitternacht',
                  noon: 'Mittag',
                  morning: 'morgens',
                  afternoon: 'nachmittags',
                  evening: 'abends',
                  night: 'nachts',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
