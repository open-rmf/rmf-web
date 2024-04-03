(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [36648, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-DZ/_lib/localize/index.js':
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
            ordinalNumber: function ordinalNumber(dirtyNumber) {
              return String(dirtyNumber);
            },
            era: (0, _index.default)({
              values: {
                narrow: ['ق', 'ب'],
                abbreviated: ['ق.م.', 'ب.م.'],
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
                return Number(quarter) - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['ج', 'ف', 'م', 'أ', 'م', 'ج', 'ج', 'أ', 'س', 'أ', 'ن', 'د'],
                abbreviated: [
                  'جانـ',
                  'فيفـ',
                  'مارس',
                  'أفريل',
                  'مايـ',
                  'جوانـ',
                  'جويـ',
                  'أوت',
                  'سبتـ',
                  'أكتـ',
                  'نوفـ',
                  'ديسـ',
                ],
                wide: [
                  'جانفي',
                  'فيفري',
                  'مارس',
                  'أفريل',
                  'ماي',
                  'جوان',
                  'جويلية',
                  'أوت',
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
                abbreviated: ['أحد', 'اثنـ', 'ثلا', 'أربـ', 'خميـ', 'جمعة', 'سبت'],
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
                  evening: 'مساءاً',
                  night: 'ليلاً',
                },
                abbreviated: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  noon: 'ظهر',
                  morning: 'صباحاً',
                  afternoon: 'بعد الظهر',
                  evening: 'مساءاً',
                  night: 'ليلاً',
                },
                wide: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  noon: 'ظهر',
                  morning: 'صباحاً',
                  afternoon: 'بعد الظهر',
                  evening: 'مساءاً',
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
                  afternoon: 'بعد الظـهر',
                  evening: 'في المساء',
                  night: 'في الليل',
                },
                abbreviated: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  noon: 'ظهر',
                  morning: 'في الصباح',
                  afternoon: 'بعد الظهر',
                  evening: 'في المساء',
                  night: 'في الليل',
                },
                wide: {
                  am: 'ص',
                  pm: 'م',
                  midnight: 'نصف الليل',
                  noon: 'ظهر',
                  morning: 'صباحاً',
                  afternoon: 'بعد الظـهر',
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
