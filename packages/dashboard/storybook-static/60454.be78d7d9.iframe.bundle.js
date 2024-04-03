(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [60454, 69107, 69421],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/formatDistance/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/localize/index.js',
          ),
          formatDistanceLocale = {
            lessThanXSeconds: { one: '१ सेकंड से कम', other: '{{count}} सेकंड से कम' },
            xSeconds: { one: '१ सेकंड', other: '{{count}} सेकंड' },
            halfAMinute: 'आधा मिनट',
            lessThanXMinutes: { one: '१ मिनट से कम', other: '{{count}} मिनट से कम' },
            xMinutes: { one: '१ मिनट', other: '{{count}} मिनट' },
            aboutXHours: { one: 'लगभग १ घंटा', other: 'लगभग {{count}} घंटे' },
            xHours: { one: '१ घंटा', other: '{{count}} घंटे' },
            xDays: { one: '१ दिन', other: '{{count}} दिन' },
            aboutXWeeks: { one: 'लगभग १ सप्ताह', other: 'लगभग {{count}} सप्ताह' },
            xWeeks: { one: '१ सप्ताह', other: '{{count}} सप्ताह' },
            aboutXMonths: { one: 'लगभग १ महीना', other: 'लगभग {{count}} महीने' },
            xMonths: { one: '१ महीना', other: '{{count}} महीने' },
            aboutXYears: { one: 'लगभग १ वर्ष', other: 'लगभग {{count}} वर्ष' },
            xYears: { one: '१ वर्ष', other: '{{count}} वर्ष' },
            overXYears: { one: '१ वर्ष से अधिक', other: '{{count}} वर्ष से अधिक' },
            almostXYears: { one: 'लगभग १ वर्ष', other: 'लगभग {{count}} वर्ष' },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : tokenValue.other.replace('{{count}}', (0, _index.numberToLocale)(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? result + 'मे '
                  : result + ' पहले'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hi/_lib/localize/index.js':
      (__unused_webpack_module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = void 0),
          (exports.localeToNumber = function localeToNumber(locale) {
            var enNumber = locale.toString().replace(/[१२३४५६७८९०]/g, function (match) {
              return numberValues.number[match];
            });
            return Number(enNumber);
          }),
          (exports.numberToLocale = numberToLocale);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildLocalizeFn/index.js',
            ),
          ),
          numberValues = {
            locale: {
              1: '१',
              2: '२',
              3: '३',
              4: '४',
              5: '५',
              6: '६',
              7: '७',
              8: '८',
              9: '९',
              0: '०',
            },
            number: {
              '१': '1',
              '२': '2',
              '३': '3',
              '४': '4',
              '५': '5',
              '६': '6',
              '७': '7',
              '८': '8',
              '९': '9',
              '०': '0',
            },
          };
        function numberToLocale(enNumber) {
          return enNumber.toString().replace(/\d/g, function (match) {
            return numberValues.locale[match];
          });
        }
        var _default = {
          ordinalNumber: function ordinalNumber(dirtyNumber, _options) {
            return numberToLocale(Number(dirtyNumber));
          },
          era: (0, _index.default)({
            values: {
              narrow: ['ईसा-पूर्व', 'ईस्वी'],
              abbreviated: ['ईसा-पूर्व', 'ईस्वी'],
              wide: ['ईसा-पूर्व', 'ईसवी सन'],
            },
            defaultWidth: 'wide',
          }),
          quarter: (0, _index.default)({
            values: {
              narrow: ['1', '2', '3', '4'],
              abbreviated: ['ति1', 'ति2', 'ति3', 'ति4'],
              wide: ['पहली तिमाही', 'दूसरी तिमाही', 'तीसरी तिमाही', 'चौथी तिमाही'],
            },
            defaultWidth: 'wide',
            argumentCallback: function argumentCallback(quarter) {
              return quarter - 1;
            },
          }),
          month: (0, _index.default)({
            values: {
              narrow: ['ज', 'फ़', 'मा', 'अ', 'मई', 'जू', 'जु', 'अग', 'सि', 'अक्टू', 'न', 'दि'],
              abbreviated: [
                'जन',
                'फ़र',
                'मार्च',
                'अप्रैल',
                'मई',
                'जून',
                'जुल',
                'अग',
                'सित',
                'अक्टू',
                'नव',
                'दिस',
              ],
              wide: [
                'जनवरी',
                'फ़रवरी',
                'मार्च',
                'अप्रैल',
                'मई',
                'जून',
                'जुलाई',
                'अगस्त',
                'सितंबर',
                'अक्टूबर',
                'नवंबर',
                'दिसंबर',
              ],
            },
            defaultWidth: 'wide',
          }),
          day: (0, _index.default)({
            values: {
              narrow: ['र', 'सो', 'मं', 'बु', 'गु', 'शु', 'श'],
              short: ['र', 'सो', 'मं', 'बु', 'गु', 'शु', 'श'],
              abbreviated: ['रवि', 'सोम', 'मंगल', 'बुध', 'गुरु', 'शुक्र', 'शनि'],
              wide: ['रविवार', 'सोमवार', 'मंगलवार', 'बुधवार', 'गुरुवार', 'शुक्रवार', 'शनिवार'],
            },
            defaultWidth: 'wide',
          }),
          dayPeriod: (0, _index.default)({
            values: {
              narrow: {
                am: 'पूर्वाह्न',
                pm: 'अपराह्न',
                midnight: 'मध्यरात्रि',
                noon: 'दोपहर',
                morning: 'सुबह',
                afternoon: 'दोपहर',
                evening: 'शाम',
                night: 'रात',
              },
              abbreviated: {
                am: 'पूर्वाह्न',
                pm: 'अपराह्न',
                midnight: 'मध्यरात्रि',
                noon: 'दोपहर',
                morning: 'सुबह',
                afternoon: 'दोपहर',
                evening: 'शाम',
                night: 'रात',
              },
              wide: {
                am: 'पूर्वाह्न',
                pm: 'अपराह्न',
                midnight: 'मध्यरात्रि',
                noon: 'दोपहर',
                morning: 'सुबह',
                afternoon: 'दोपहर',
                evening: 'शाम',
                night: 'रात',
              },
            },
            defaultWidth: 'wide',
            formattingValues: {
              narrow: {
                am: 'पूर्वाह्न',
                pm: 'अपराह्न',
                midnight: 'मध्यरात्रि',
                noon: 'दोपहर',
                morning: 'सुबह',
                afternoon: 'दोपहर',
                evening: 'शाम',
                night: 'रात',
              },
              abbreviated: {
                am: 'पूर्वाह्न',
                pm: 'अपराह्न',
                midnight: 'मध्यरात्रि',
                noon: 'दोपहर',
                morning: 'सुबह',
                afternoon: 'दोपहर',
                evening: 'शाम',
                night: 'रात',
              },
              wide: {
                am: 'पूर्वाह्न',
                pm: 'अपराह्न',
                midnight: 'मध्यरात्रि',
                noon: 'दोपहर',
                morning: 'सुबह',
                afternoon: 'दोपहर',
                evening: 'शाम',
                night: 'रात',
              },
            },
            defaultFormattingWidth: 'wide',
          }),
        };
        exports.default = _default;
      },
  },
]);
