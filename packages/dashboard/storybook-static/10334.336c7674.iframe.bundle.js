(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [10334, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/_lib/localize/index.js':
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
              var number = Number(dirtyNumber);
              return 'hour' === (null == options ? void 0 : options.unit)
                ? String(number)
                : 1 === number
                  ? number + '-rë'
                  : 4 === number
                    ? number + 't'
                    : number + '-të';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['P', 'M'],
                abbreviated: ['PK', 'MK'],
                wide: ['Para Krishtit', 'Mbas Krishtit'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['4-mujori I', '4-mujori II', '4-mujori III', '4-mujori IV'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['J', 'S', 'M', 'P', 'M', 'Q', 'K', 'G', 'S', 'T', 'N', 'D'],
                abbreviated: [
                  'Jan',
                  'Shk',
                  'Mar',
                  'Pri',
                  'Maj',
                  'Qer',
                  'Kor',
                  'Gus',
                  'Sht',
                  'Tet',
                  'Nën',
                  'Dhj',
                ],
                wide: [
                  'Janar',
                  'Shkurt',
                  'Mars',
                  'Prill',
                  'Maj',
                  'Qershor',
                  'Korrik',
                  'Gusht',
                  'Shtator',
                  'Tetor',
                  'Nëntor',
                  'Dhjetor',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['D', 'H', 'M', 'M', 'E', 'P', 'S'],
                short: ['Di', 'Hë', 'Ma', 'Më', 'En', 'Pr', 'Sh'],
                abbreviated: ['Die', 'Hën', 'Mar', 'Mër', 'Enj', 'Pre', 'Sht'],
                wide: ['Dielë', 'Hënë', 'Martë', 'Mërkurë', 'Enjte', 'Premte', 'Shtunë'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'p',
                  pm: 'm',
                  midnight: 'm',
                  noon: 'd',
                  morning: 'mëngjes',
                  afternoon: 'dite',
                  evening: 'mbrëmje',
                  night: 'natë',
                },
                abbreviated: {
                  am: 'PD',
                  pm: 'MD',
                  midnight: 'mesnëtë',
                  noon: 'drek',
                  morning: 'mëngjes',
                  afternoon: 'mbasdite',
                  evening: 'mbrëmje',
                  night: 'natë',
                },
                wide: {
                  am: 'p.d.',
                  pm: 'm.d.',
                  midnight: 'mesnëtë',
                  noon: 'drek',
                  morning: 'mëngjes',
                  afternoon: 'mbasdite',
                  evening: 'mbrëmje',
                  night: 'natë',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'p',
                  pm: 'm',
                  midnight: 'm',
                  noon: 'd',
                  morning: 'në mëngjes',
                  afternoon: 'në mbasdite',
                  evening: 'në mbrëmje',
                  night: 'në mesnatë',
                },
                abbreviated: {
                  am: 'PD',
                  pm: 'MD',
                  midnight: 'mesnatë',
                  noon: 'drek',
                  morning: 'në mëngjes',
                  afternoon: 'në mbasdite',
                  evening: 'në mbrëmje',
                  night: 'në mesnatë',
                },
                wide: {
                  am: 'p.d.',
                  pm: 'm.d.',
                  midnight: 'mesnatë',
                  noon: 'drek',
                  morning: 'në mëngjes',
                  afternoon: 'në mbasdite',
                  evening: 'në mbrëmje',
                  night: 'në mesnatë',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
