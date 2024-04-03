(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [88775, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/_lib/localize/index.js':
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
              return Number(dirtyNumber) + 'e';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['f.K.', 'n.K.'],
                abbreviated: ['f.Kr.', 'n.Kr.'],
                wide: ['foar Kristus', 'nei Kristus'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['K1', 'K2', 'K3', 'K4'],
                wide: ['1e fearnsjier', '2e fearnsjier', '3e fearnsjier', '4e fearnsjier'],
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
                  'mrt.',
                  'apr.',
                  'mai.',
                  'jun.',
                  'jul.',
                  'aug.',
                  'sep.',
                  'okt.',
                  'nov.',
                  'des.',
                ],
                wide: [
                  'jannewaris',
                  'febrewaris',
                  'maart',
                  'april',
                  'maaie',
                  'juny',
                  'july',
                  'augustus',
                  'septimber',
                  'oktober',
                  'novimber',
                  'desimber',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['s', 'm', 't', 'w', 't', 'f', 's'],
                short: ['si', 'mo', 'ti', 'wo', 'to', 'fr', 'so'],
                abbreviated: ['snein', 'moa', 'tii', 'woa', 'ton', 'fre', 'sneon'],
                wide: ['snein', 'moandei', 'tiisdei', 'woansdei', 'tongersdei', 'freed', 'sneon'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'middernacht',
                  noon: 'middei',
                  morning: 'moarns',
                  afternoon: 'middeis',
                  evening: 'jûns',
                  night: 'nachts',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'middernacht',
                  noon: 'middei',
                  morning: 'moarns',
                  afternoon: 'middeis',
                  evening: 'jûns',
                  night: 'nachts',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'middernacht',
                  noon: 'middei',
                  morning: 'moarns',
                  afternoon: 'middeis',
                  evening: 'jûns',
                  night: 'nachts',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
