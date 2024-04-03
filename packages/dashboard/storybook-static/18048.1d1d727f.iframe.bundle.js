(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [18048, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/localize/index.js':
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
            ordinalNumber: function ordinalNumber(dirtyNumber, options) {
              var ordinal,
                number = Number(dirtyNumber),
                unit = null == options ? void 0 : options.unit;
              switch (number) {
                case 1:
                  ordinal = 'èr';
                  break;
                case 2:
                  ordinal = 'nd';
                  break;
                default:
                  ordinal = 'en';
              }
              return (
                ('year' !== unit &&
                  'week' !== unit &&
                  'hour' !== unit &&
                  'minute' !== unit &&
                  'second' !== unit) ||
                  (ordinal += 'a'),
                number + ordinal
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['ab. J.C.', 'apr. J.C.'],
                abbreviated: ['ab. J.C.', 'apr. J.C.'],
                wide: ['abans Jèsus-Crist', 'après Jèsus-Crist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['T1', 'T2', 'T3', 'T4'],
                abbreviated: ['1èr trim.', '2nd trim.', '3en trim.', '4en trim.'],
                wide: ['1èr trimèstre', '2nd trimèstre', '3en trimèstre', '4en trimèstre'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['GN', 'FB', 'MÇ', 'AB', 'MA', 'JN', 'JL', 'AG', 'ST', 'OC', 'NV', 'DC'],
                abbreviated: [
                  'gen.',
                  'febr.',
                  'març',
                  'abr.',
                  'mai',
                  'junh',
                  'jul.',
                  'ag.',
                  'set.',
                  'oct.',
                  'nov.',
                  'dec.',
                ],
                wide: [
                  'genièr',
                  'febrièr',
                  'març',
                  'abril',
                  'mai',
                  'junh',
                  'julhet',
                  'agost',
                  'setembre',
                  'octòbre',
                  'novembre',
                  'decembre',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['dg.', 'dl.', 'dm.', 'dc.', 'dj.', 'dv.', 'ds.'],
                short: ['dg.', 'dl.', 'dm.', 'dc.', 'dj.', 'dv.', 'ds.'],
                abbreviated: ['dg.', 'dl.', 'dm.', 'dc.', 'dj.', 'dv.', 'ds.'],
                wide: [
                  'dimenge',
                  'diluns',
                  'dimars',
                  'dimècres',
                  'dijòus',
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
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'matin',
                  afternoon: 'aprèp-miègjorn',
                  evening: 'vèspre',
                  night: 'nuèch',
                },
                abbreviated: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'matin',
                  afternoon: 'aprèp-miègjorn',
                  evening: 'vèspre',
                  night: 'nuèch',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'matin',
                  afternoon: 'aprèp-miègjorn',
                  evening: 'vèspre',
                  night: 'nuèch',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'am',
                  pm: 'pm',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'del matin',
                  afternoon: 'de l’aprèp-miègjorn',
                  evening: 'del ser',
                  night: 'de la nuèch',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'del matin',
                  afternoon: 'de l’aprèp-miègjorn',
                  evening: 'del ser',
                  night: 'de la nuèch',
                },
                wide: {
                  am: 'ante meridiem',
                  pm: 'post meridiem',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'del matin',
                  afternoon: 'de l’aprèp-miègjorn',
                  evening: 'del ser',
                  night: 'de la nuèch',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
