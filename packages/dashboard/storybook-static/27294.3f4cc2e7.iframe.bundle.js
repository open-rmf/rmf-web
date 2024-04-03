(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [27294, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/localize/index.js':
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
              var number = Number(dirtyNumber),
                rem100 = number % 100;
              if (rem100 > 20 || rem100 < 10)
                switch (rem100 % 10) {
                  case 1:
                  case 3:
                    return number + 'r';
                  case 2:
                    return number + 'n';
                  case 4:
                    return number + 't';
                }
              return number + 'è';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['aC', 'dC'],
                abbreviated: ['a. de C.', 'd. de C.'],
                wide: ['abans de Crist', 'després de Crist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['T1', 'T2', 'T3', 'T4'],
                wide: ['1r trimestre', '2n trimestre', '3r trimestre', '4t trimestre'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['GN', 'FB', 'MÇ', 'AB', 'MG', 'JN', 'JL', 'AG', 'ST', 'OC', 'NV', 'DS'],
                abbreviated: [
                  'gen.',
                  'febr.',
                  'març',
                  'abr.',
                  'maig',
                  'juny',
                  'jul.',
                  'ag.',
                  'set.',
                  'oct.',
                  'nov.',
                  'des.',
                ],
                wide: [
                  'gener',
                  'febrer',
                  'març',
                  'abril',
                  'maig',
                  'juny',
                  'juliol',
                  'agost',
                  'setembre',
                  'octubre',
                  'novembre',
                  'desembre',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['dg.', 'dl.', 'dt.', 'dm.', 'dj.', 'dv.', 'ds.'],
                short: ['dg.', 'dl.', 'dt.', 'dm.', 'dj.', 'dv.', 'ds.'],
                abbreviated: ['dg.', 'dl.', 'dt.', 'dm.', 'dj.', 'dv.', 'ds.'],
                wide: [
                  'diumenge',
                  'dilluns',
                  'dimarts',
                  'dimecres',
                  'dijous',
                  'divendres',
                  'dissabte',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'am',
                  pm: 'pm',
                  midnight: 'mitjanit',
                  noon: 'migdia',
                  morning: 'matí',
                  afternoon: 'tarda',
                  evening: 'vespre',
                  night: 'nit',
                },
                abbreviated: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'mitjanit',
                  noon: 'migdia',
                  morning: 'matí',
                  afternoon: 'tarda',
                  evening: 'vespre',
                  night: 'nit',
                },
                wide: {
                  am: 'ante meridiem',
                  pm: 'post meridiem',
                  midnight: 'mitjanit',
                  noon: 'migdia',
                  morning: 'matí',
                  afternoon: 'tarda',
                  evening: 'vespre',
                  night: 'nit',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'am',
                  pm: 'pm',
                  midnight: 'de la mitjanit',
                  noon: 'del migdia',
                  morning: 'del matí',
                  afternoon: 'de la tarda',
                  evening: 'del vespre',
                  night: 'de la nit',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'de la mitjanit',
                  noon: 'del migdia',
                  morning: 'del matí',
                  afternoon: 'de la tarda',
                  evening: 'del vespre',
                  night: 'de la nit',
                },
                wide: {
                  am: 'ante meridiem',
                  pm: 'post meridiem',
                  midnight: 'de la mitjanit',
                  noon: 'del migdia',
                  morning: 'del matí',
                  afternoon: 'de la tarda',
                  evening: 'del vespre',
                  night: 'de la nit',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
