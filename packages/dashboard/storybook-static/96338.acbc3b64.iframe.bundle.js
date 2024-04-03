(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [96338, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/_lib/localize/index.js':
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
                narrow: ['ق', 'ب'],
                abbreviated: ['ق.م', 'ب.م'],
                wide: ['قبل الميلاد', 'بعد الميلاد'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['ر1', 'ر2', 'ر3', 'ر4'],
                wide: ['الربع الأول', 'الربع الثاني', 'الربع الثالث', 'الربع الرابع'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['ي', 'ف', 'م', 'أ', 'م', 'ي', 'ي', 'أ', 'س', 'أ', 'ن', 'د'],
                abbreviated: [
                  'ينا',
                  'فبر',
                  'مارس',
                  'أبريل',
                  'مايو',
                  'يونـ',
                  'يولـ',
                  'أغسـ',
                  'سبتـ',
                  'أكتـ',
                  'نوفـ',
                  'ديسـ',
                ],
                wide: [
                  'يناير',
                  'فبراير',
                  'مارس',
                  'أبريل',
                  'مايو',
                  'يونيو',
                  'يوليو',
                  'أغسطس',
                  'سبتمبر',
                  'أكتوبر',
                  'نوفمبر',
                  'ديسمبر',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ح', 'ن', 'ث', 'ر', 'خ', 'ج', 'س'],
                short: ['أحد', 'اثنين', 'ثلاثاء', 'أربعاء', 'خميس', 'جمعة', 'سبت'],
                abbreviated: ['أحد', 'اثنين', 'ثلاثاء', 'أربعاء', 'خميس', 'جمعة', 'سبت'],
                wide: ['الأحد', 'الاثنين', 'الثلاثاء', 'الأربعاء', 'الخميس', 'الجمعة', 'السبت'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'ن',
                  noon: 'ظ',
                  morning: 'صباحاً',
                  afternoon: 'بعد الظهر',
                  evening: 'مساءً',
                  night: 'ليلاً',
                },
                abbreviated: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  noon: 'ظهراً',
                  morning: 'صباحاً',
                  afternoon: 'بعد الظهر',
                  evening: 'مساءً',
                  night: 'ليلاً',
                },
                wide: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  noon: 'ظهراً',
                  morning: 'صباحاً',
                  afternoon: 'بعد الظهر',
                  evening: 'مساءً',
                  night: 'ليلاً',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'ن',
                  noon: 'ظ',
                  morning: 'في الصباح',
                  afternoon: 'بعد الظهر',
                  evening: 'في المساء',
                  night: 'في الليل',
                },
                abbreviated: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  noon: 'ظهراً',
                  morning: 'في الصباح',
                  afternoon: 'بعد الظهر',
                  evening: 'في المساء',
                  night: 'في الليل',
                },
                wide: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  morning: 'في الصباح',
                  noon: 'ظهراً',
                  afternoon: 'بعد الظهر',
                  evening: 'في المساء',
                  night: 'في الليل',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
