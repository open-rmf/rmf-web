(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [5558, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/localize/index.js':
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
              var number = Number(dirtyNumber);
              if (number < 20)
                switch (number) {
                  case 0:
                  case 7:
                  case 8:
                  case 9:
                  case 10:
                  case 12:
                  case 15:
                  case 18:
                    return number + 'fed';
                  case 1:
                    return number + 'af';
                  case 2:
                    return number + 'ail';
                  case 3:
                  case 4:
                    return number + 'ydd';
                  case 5:
                  case 6:
                    return number + 'ed';
                  case 11:
                  case 13:
                  case 14:
                  case 16:
                  case 17:
                  case 19:
                    return number + 'eg';
                }
              else if ((number >= 50 && number <= 60) || 80 === number || number >= 100)
                return number + 'fed';
              return number + 'ain';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['C', 'O'],
                abbreviated: ['CC', 'OC'],
                wide: ['Cyn Crist', 'Ar ôl Crist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Ch1', 'Ch2', 'Ch3', 'Ch4'],
                wide: ['Chwarter 1af', '2ail chwarter', '3ydd chwarter', '4ydd chwarter'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['I', 'Ch', 'Ma', 'E', 'Mi', 'Me', 'G', 'A', 'Md', 'H', 'T', 'Rh'],
                abbreviated: [
                  'Ion',
                  'Chwe',
                  'Maw',
                  'Ebr',
                  'Mai',
                  'Meh',
                  'Gor',
                  'Aws',
                  'Med',
                  'Hyd',
                  'Tach',
                  'Rhag',
                ],
                wide: [
                  'Ionawr',
                  'Chwefror',
                  'Mawrth',
                  'Ebrill',
                  'Mai',
                  'Mehefin',
                  'Gorffennaf',
                  'Awst',
                  'Medi',
                  'Hydref',
                  'Tachwedd',
                  'Rhagfyr',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['S', 'Ll', 'M', 'M', 'I', 'G', 'S'],
                short: ['Su', 'Ll', 'Ma', 'Me', 'Ia', 'Gw', 'Sa'],
                abbreviated: ['Sul', 'Llun', 'Maw', 'Mer', 'Iau', 'Gwe', 'Sad'],
                wide: [
                  'dydd Sul',
                  'dydd Llun',
                  'dydd Mawrth',
                  'dydd Mercher',
                  'dydd Iau',
                  'dydd Gwener',
                  'dydd Sadwrn',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'b',
                  pm: 'h',
                  midnight: 'hn',
                  noon: 'hd',
                  morning: 'bore',
                  afternoon: 'prynhawn',
                  evening: "gyda'r nos",
                  night: 'nos',
                },
                abbreviated: {
                  am: 'yb',
                  pm: 'yh',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'bore',
                  afternoon: 'prynhawn',
                  evening: "gyda'r nos",
                  night: 'nos',
                },
                wide: {
                  am: 'y.b.',
                  pm: 'y.h.',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'bore',
                  afternoon: 'prynhawn',
                  evening: "gyda'r nos",
                  night: 'nos',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'b',
                  pm: 'h',
                  midnight: 'hn',
                  noon: 'hd',
                  morning: 'yn y bore',
                  afternoon: 'yn y prynhawn',
                  evening: "gyda'r nos",
                  night: 'yn y nos',
                },
                abbreviated: {
                  am: 'yb',
                  pm: 'yh',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'yn y bore',
                  afternoon: 'yn y prynhawn',
                  evening: "gyda'r nos",
                  night: 'yn y nos',
                },
                wide: {
                  am: 'y.b.',
                  pm: 'y.h.',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'yn y bore',
                  afternoon: 'yn y prynhawn',
                  evening: "gyda'r nos",
                  night: 'yn y nos',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
