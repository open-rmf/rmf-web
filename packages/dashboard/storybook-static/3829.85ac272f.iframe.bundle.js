(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [3829, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/localize/index.js':
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
              return Number(dirtyNumber) + 'º';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['AC', 'DC'],
                abbreviated: ['AC', 'DC'],
                wide: ['antes de cristo', 'despois de cristo'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['T1', 'T2', 'T3', 'T4'],
                wide: ['1º trimestre', '2º trimestre', '3º trimestre', '4º trimestre'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['e', 'f', 'm', 'a', 'm', 'j', 'j', 'a', 's', 'o', 'n', 'd'],
                abbreviated: [
                  'xan',
                  'feb',
                  'mar',
                  'abr',
                  'mai',
                  'xun',
                  'xul',
                  'ago',
                  'set',
                  'out',
                  'nov',
                  'dec',
                ],
                wide: [
                  'xaneiro',
                  'febreiro',
                  'marzo',
                  'abril',
                  'maio',
                  'xuño',
                  'xullo',
                  'agosto',
                  'setembro',
                  'outubro',
                  'novembro',
                  'decembro',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['d', 'l', 'm', 'm', 'j', 'v', 's'],
                short: ['do', 'lu', 'ma', 'me', 'xo', 've', 'sa'],
                abbreviated: ['dom', 'lun', 'mar', 'mer', 'xov', 'ven', 'sab'],
                wide: ['domingo', 'luns', 'martes', 'mércores', 'xoves', 'venres', 'sábado'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'mn',
                  noon: 'md',
                  morning: 'mañá',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noite',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'medianoite',
                  noon: 'mediodía',
                  morning: 'mañá',
                  afternoon: 'tarde',
                  evening: 'tardiña',
                  night: 'noite',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'medianoite',
                  noon: 'mediodía',
                  morning: 'mañá',
                  afternoon: 'tarde',
                  evening: 'tardiña',
                  night: 'noite',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'mn',
                  noon: 'md',
                  morning: 'da mañá',
                  afternoon: 'da tarde',
                  evening: 'da tardiña',
                  night: 'da noite',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'medianoite',
                  noon: 'mediodía',
                  morning: 'da mañá',
                  afternoon: 'da tarde',
                  evening: 'da tardiña',
                  night: 'da noite',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'medianoite',
                  noon: 'mediodía',
                  morning: 'da mañá',
                  afternoon: 'da tarde',
                  evening: 'da tardiña',
                  night: 'da noite',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
