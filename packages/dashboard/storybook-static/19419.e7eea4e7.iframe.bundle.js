(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [19419, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/_lib/localize/index.js':
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
              return String(number);
            },
            era: (0, _index.default)({
              values: {
                narrow: ['aC', 'dC'],
                abbreviated: ['a.C.', 'd.C.'],
                wide: ['avanti Cristo', 'dopo Cristo'],
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
                narrow: ['G', 'F', 'M', 'A', 'M', 'G', 'L', 'A', 'S', 'O', 'N', 'D'],
                abbreviated: [
                  'gen',
                  'feb',
                  'mar',
                  'apr',
                  'mag',
                  'giu',
                  'lug',
                  'ago',
                  'set',
                  'ott',
                  'nov',
                  'dic',
                ],
                wide: [
                  'gennaio',
                  'febbraio',
                  'marzo',
                  'aprile',
                  'maggio',
                  'giugno',
                  'luglio',
                  'agosto',
                  'settembre',
                  'ottobre',
                  'novembre',
                  'dicembre',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['D', 'L', 'M', 'M', 'G', 'V', 'S'],
                short: ['dom', 'lun', 'mar', 'mer', 'gio', 'ven', 'sab'],
                abbreviated: ['dom', 'lun', 'mar', 'mer', 'gio', 'ven', 'sab'],
                wide: [
                  'domenica',
                  'lunedì',
                  'martedì',
                  'mercoledì',
                  'giovedì',
                  'venerdì',
                  'sabato',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'm.',
                  pm: 'p.',
                  midnight: 'mezzanotte',
                  noon: 'mezzogiorno',
                  morning: 'mattina',
                  afternoon: 'pomeriggio',
                  evening: 'sera',
                  night: 'notte',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'mezzanotte',
                  noon: 'mezzogiorno',
                  morning: 'mattina',
                  afternoon: 'pomeriggio',
                  evening: 'sera',
                  night: 'notte',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'mezzanotte',
                  noon: 'mezzogiorno',
                  morning: 'mattina',
                  afternoon: 'pomeriggio',
                  evening: 'sera',
                  night: 'notte',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'm.',
                  pm: 'p.',
                  midnight: 'mezzanotte',
                  noon: 'mezzogiorno',
                  morning: 'di mattina',
                  afternoon: 'del pomeriggio',
                  evening: 'di sera',
                  night: 'di notte',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'mezzanotte',
                  noon: 'mezzogiorno',
                  morning: 'di mattina',
                  afternoon: 'del pomeriggio',
                  evening: 'di sera',
                  night: 'di notte',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'mezzanotte',
                  noon: 'mezzogiorno',
                  morning: 'di mattina',
                  afternoon: 'del pomeriggio',
                  evening: 'di sera',
                  night: 'di notte',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
