(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [20306, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/localize/index.js':
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
              return String(dirtyNumber);
            },
            era: (0, _index.default)({
              values: {
                narrow: ['ب', 'ك'],
                abbreviated: ['ب', 'ك'],
                wide: ['مىيلادىدىن بۇرۇن', 'مىيلادىدىن كىيىن'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1', '2', '3', '4'],
                wide: ['بىرىنجى چارەك', 'ئىككىنجى چارەك', 'ئۈچىنجى چارەك', 'تۆتىنجى چارەك'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['ي', 'ف', 'م', 'ا', 'م', 'ى', 'ى', 'ا', 'س', 'ۆ', 'ن', 'د'],
                abbreviated: [
                  'يانۋار',
                  'فېۋىرال',
                  'مارت',
                  'ئاپرىل',
                  'ماي',
                  'ئىيۇن',
                  'ئىيول',
                  'ئاۋغۇست',
                  'سىنتەبىر',
                  'ئۆكتەبىر',
                  'نويابىر',
                  'دىكابىر',
                ],
                wide: [
                  'يانۋار',
                  'فېۋىرال',
                  'مارت',
                  'ئاپرىل',
                  'ماي',
                  'ئىيۇن',
                  'ئىيول',
                  'ئاۋغۇست',
                  'سىنتەبىر',
                  'ئۆكتەبىر',
                  'نويابىر',
                  'دىكابىر',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ي', 'د', 'س', 'چ', 'پ', 'ج', 'ش'],
                short: ['ي', 'د', 'س', 'چ', 'پ', 'ج', 'ش'],
                abbreviated: [
                  'يەكشەنبە',
                  'دۈشەنبە',
                  'سەيشەنبە',
                  'چارشەنبە',
                  'پەيشەنبە',
                  'جۈمە',
                  'شەنبە',
                ],
                wide: ['يەكشەنبە', 'دۈشەنبە', 'سەيشەنبە', 'چارشەنبە', 'پەيشەنبە', 'جۈمە', 'شەنبە'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ئە',
                  pm: 'چ',
                  midnight: 'ك',
                  noon: 'چ',
                  morning: 'ئەتىگەن',
                  afternoon: 'چۈشتىن كىيىن',
                  evening: 'ئاخشىم',
                  night: 'كىچە',
                },
                abbreviated: {
                  am: 'ئە',
                  pm: 'چ',
                  midnight: 'ك',
                  noon: 'چ',
                  morning: 'ئەتىگەن',
                  afternoon: 'چۈشتىن كىيىن',
                  evening: 'ئاخشىم',
                  night: 'كىچە',
                },
                wide: {
                  am: 'ئە',
                  pm: 'چ',
                  midnight: 'ك',
                  noon: 'چ',
                  morning: 'ئەتىگەن',
                  afternoon: 'چۈشتىن كىيىن',
                  evening: 'ئاخشىم',
                  night: 'كىچە',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'ئە',
                  pm: 'چ',
                  midnight: 'ك',
                  noon: 'چ',
                  morning: 'ئەتىگەندە',
                  afternoon: 'چۈشتىن كىيىن',
                  evening: 'ئاخشامدا',
                  night: 'كىچىدە',
                },
                abbreviated: {
                  am: 'ئە',
                  pm: 'چ',
                  midnight: 'ك',
                  noon: 'چ',
                  morning: 'ئەتىگەندە',
                  afternoon: 'چۈشتىن كىيىن',
                  evening: 'ئاخشامدا',
                  night: 'كىچىدە',
                },
                wide: {
                  am: 'ئە',
                  pm: 'چ',
                  midnight: 'ك',
                  noon: 'چ',
                  morning: 'ئەتىگەندە',
                  afternoon: 'چۈشتىن كىيىن',
                  evening: 'ئاخشامدا',
                  night: 'كىچىدە',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
