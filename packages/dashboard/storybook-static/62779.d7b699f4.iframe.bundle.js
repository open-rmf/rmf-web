(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [62779, 45585, 69107, 95565, 79489, 38157, 93448, 59114, 30002, 63606],
  {
    '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js':
      (module) => {
        (module.exports = function _interopRequireDefault(obj) {
          return obj && obj.__esModule ? obj : { default: obj };
        }),
          (module.exports.__esModule = !0),
          (module.exports.default = module.exports);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildFormatLongFn/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = function buildFormatLongFn(args) {
            return function () {
              var options = arguments.length > 0 && void 0 !== arguments[0] ? arguments[0] : {},
                width = options.width ? String(options.width) : args.defaultWidth;
              return args.formats[width] || args.formats[args.defaultWidth];
            };
          }),
          (module.exports = exports.default);
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchFn/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = function buildMatchFn(args) {
            return function (string) {
              var options = arguments.length > 1 && void 0 !== arguments[1] ? arguments[1] : {},
                width = options.width,
                matchPattern =
                  (width && args.matchPatterns[width]) ||
                  args.matchPatterns[args.defaultMatchWidth],
                matchResult = string.match(matchPattern);
              if (!matchResult) return null;
              var value,
                matchedString = matchResult[0],
                parsePatterns =
                  (width && args.parsePatterns[width]) ||
                  args.parsePatterns[args.defaultParseWidth],
                key = Array.isArray(parsePatterns)
                  ? (function findIndex(array, predicate) {
                      for (var key = 0; key < array.length; key++)
                        if (predicate(array[key])) return key;
                      return;
                    })(parsePatterns, function (pattern) {
                      return pattern.test(matchedString);
                    })
                  : (function findKey(object, predicate) {
                      for (var key in object)
                        if (object.hasOwnProperty(key) && predicate(object[key])) return key;
                      return;
                    })(parsePatterns, function (pattern) {
                      return pattern.test(matchedString);
                    });
              return (
                (value = args.valueCallback ? args.valueCallback(key) : key),
                {
                  value: (value = options.valueCallback ? options.valueCallback(value) : value),
                  rest: string.slice(matchedString.length),
                }
              );
            };
          }),
          (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchPatternFn/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = function buildMatchPatternFn(args) {
            return function (string) {
              var options = arguments.length > 1 && void 0 !== arguments[1] ? arguments[1] : {},
                matchResult = string.match(args.matchPattern);
              if (!matchResult) return null;
              var matchedString = matchResult[0],
                parseResult = string.match(args.parsePattern);
              if (!parseResult) return null;
              var value = args.valueCallback ? args.valueCallback(parseResult[0]) : parseResult[0];
              return {
                value: (value = options.valueCallback ? options.valueCallback(value) : value),
                rest: string.slice(matchedString.length),
              };
            };
          }),
          (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatDistance/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/localize/index.js',
          ),
          formatDistanceLocale = {
            lessThanXSeconds: { one: 'প্রায় ১ সেকেন্ড', other: 'প্রায় {{count}} সেকেন্ড' },
            xSeconds: { one: '১ সেকেন্ড', other: '{{count}} সেকেন্ড' },
            halfAMinute: 'আধ মিনিট',
            lessThanXMinutes: { one: 'প্রায় ১ মিনিট', other: 'প্রায় {{count}} মিনিট' },
            xMinutes: { one: '১ মিনিট', other: '{{count}} মিনিট' },
            aboutXHours: { one: 'প্রায় ১ ঘন্টা', other: 'প্রায় {{count}} ঘন্টা' },
            xHours: { one: '১ ঘন্টা', other: '{{count}} ঘন্টা' },
            xDays: { one: '১ দিন', other: '{{count}} দিন' },
            aboutXWeeks: { one: 'প্রায় ১ সপ্তাহ', other: 'প্রায় {{count}} সপ্তাহ' },
            xWeeks: { one: '১ সপ্তাহ', other: '{{count}} সপ্তাহ' },
            aboutXMonths: { one: 'প্রায় ১ মাস', other: 'প্রায় {{count}} মাস' },
            xMonths: { one: '১ মাস', other: '{{count}} মাস' },
            aboutXYears: { one: 'প্রায় ১ বছর', other: 'প্রায় {{count}} বছর' },
            xYears: { one: '১ বছর', other: '{{count}} বছর' },
            overXYears: { one: '১ বছরের বেশি', other: '{{count}} বছরের বেশি' },
            almostXYears: { one: 'প্রায় ১ বছর', other: 'প্রায় {{count}} বছর' },
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
                  ? result + ' এর মধ্যে'
                  : result + ' আগে'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatLong/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildFormatLongFn/index.js',
            ),
          ),
          _default = {
            date: (0, _index.default)({
              formats: {
                full: 'EEEE, MMMM do, y',
                long: 'MMMM do, y',
                medium: 'MMM d, y',
                short: 'MM/dd/yyyy',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'h:mm:ss a zzzz',
                long: 'h:mm:ss a z',
                medium: 'h:mm:ss a',
                short: 'h:mm a',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: "{{date}} {{time}} 'সময়'",
                long: "{{date}} {{time}} 'সময়'",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'গত' eeee 'সময়' p",
            yesterday: "'গতকাল' 'সময়' p",
            today: "'আজ' 'সময়' p",
            tomorrow: "'আগামীকাল' 'সময়' p",
            nextWeek: "eeee 'সময়' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/localize/index.js':
      (__unused_webpack_module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = void 0),
          (exports.numberToLocale = numberToLocale);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildLocalizeFn/index.js',
            ),
          ),
          numberValues = {
            locale: {
              1: '১',
              2: '২',
              3: '৩',
              4: '৪',
              5: '৫',
              6: '৬',
              7: '৭',
              8: '৮',
              9: '৯',
              0: '০',
            },
            number: {
              '১': '1',
              '২': '2',
              '৩': '3',
              '৪': '4',
              '৫': '5',
              '৬': '6',
              '৭': '7',
              '৮': '8',
              '৯': '9',
              '০': '0',
            },
          };
        function numberToLocale(enNumber) {
          return enNumber.toString().replace(/\d/g, function (match) {
            return numberValues.locale[match];
          });
        }
        var _default = {
          ordinalNumber: function ordinalNumber(dirtyNumber, options) {
            var number = Number(dirtyNumber),
              localeNumber = numberToLocale(number);
            if ('date' === (null == options ? void 0 : options.unit))
              return (function dateOrdinalNumber(number, localeNumber) {
                if (number > 18 && number <= 31) return localeNumber + 'শে';
                switch (number) {
                  case 1:
                    return localeNumber + 'লা';
                  case 2:
                  case 3:
                    return localeNumber + 'রা';
                  case 4:
                    return localeNumber + 'ঠা';
                  default:
                    return localeNumber + 'ই';
                }
              })(number, localeNumber);
            if (number > 10 || 0 === number) return localeNumber + 'তম';
            switch (number % 10) {
              case 2:
              case 3:
                return localeNumber + 'য়';
              case 4:
                return localeNumber + 'র্থ';
              case 6:
                return localeNumber + 'ষ্ঠ';
              default:
                return localeNumber + 'ম';
            }
          },
          era: (0, _index.default)({
            values: {
              narrow: ['খ্রিঃপূঃ', 'খ্রিঃ'],
              abbreviated: ['খ্রিঃপূর্ব', 'খ্রিঃ'],
              wide: ['খ্রিস্টপূর্ব', 'খ্রিস্টাব্দ'],
            },
            defaultWidth: 'wide',
          }),
          quarter: (0, _index.default)({
            values: {
              narrow: ['১', '২', '৩', '৪'],
              abbreviated: ['১ত্রৈ', '২ত্রৈ', '৩ত্রৈ', '৪ত্রৈ'],
              wide: ['১ম ত্রৈমাসিক', '২য় ত্রৈমাসিক', '৩য় ত্রৈমাসিক', '৪র্থ ত্রৈমাসিক'],
            },
            defaultWidth: 'wide',
            argumentCallback: function argumentCallback(quarter) {
              return quarter - 1;
            },
          }),
          month: (0, _index.default)({
            values: {
              narrow: [
                'জানু',
                'ফেব্রু',
                'মার্চ',
                'এপ্রিল',
                'মে',
                'জুন',
                'জুলাই',
                'আগস্ট',
                'সেপ্ট',
                'অক্টো',
                'নভে',
                'ডিসে',
              ],
              abbreviated: [
                'জানু',
                'ফেব্রু',
                'মার্চ',
                'এপ্রিল',
                'মে',
                'জুন',
                'জুলাই',
                'আগস্ট',
                'সেপ্ট',
                'অক্টো',
                'নভে',
                'ডিসে',
              ],
              wide: [
                'জানুয়ারি',
                'ফেব্রুয়ারি',
                'মার্চ',
                'এপ্রিল',
                'মে',
                'জুন',
                'জুলাই',
                'আগস্ট',
                'সেপ্টেম্বর',
                'অক্টোবর',
                'নভেম্বর',
                'ডিসেম্বর',
              ],
            },
            defaultWidth: 'wide',
          }),
          day: (0, _index.default)({
            values: {
              narrow: ['র', 'সো', 'ম', 'বু', 'বৃ', 'শু', 'শ'],
              short: ['রবি', 'সোম', 'মঙ্গল', 'বুধ', 'বৃহ', 'শুক্র', 'শনি'],
              abbreviated: ['রবি', 'সোম', 'মঙ্গল', 'বুধ', 'বৃহ', 'শুক্র', 'শনি'],
              wide: [
                'রবিবার',
                'সোমবার',
                'মঙ্গলবার',
                'বুধবার',
                'বৃহস্পতিবার ',
                'শুক্রবার',
                'শনিবার',
              ],
            },
            defaultWidth: 'wide',
          }),
          dayPeriod: (0, _index.default)({
            values: {
              narrow: {
                am: 'পূ',
                pm: 'অপ',
                midnight: 'মধ্যরাত',
                noon: 'মধ্যাহ্ন',
                morning: 'সকাল',
                afternoon: 'বিকাল',
                evening: 'সন্ধ্যা',
                night: 'রাত',
              },
              abbreviated: {
                am: 'পূর্বাহ্ন',
                pm: 'অপরাহ্ন',
                midnight: 'মধ্যরাত',
                noon: 'মধ্যাহ্ন',
                morning: 'সকাল',
                afternoon: 'বিকাল',
                evening: 'সন্ধ্যা',
                night: 'রাত',
              },
              wide: {
                am: 'পূর্বাহ্ন',
                pm: 'অপরাহ্ন',
                midnight: 'মধ্যরাত',
                noon: 'মধ্যাহ্ন',
                morning: 'সকাল',
                afternoon: 'বিকাল',
                evening: 'সন্ধ্যা',
                night: 'রাত',
              },
            },
            defaultWidth: 'wide',
            formattingValues: {
              narrow: {
                am: 'পূ',
                pm: 'অপ',
                midnight: 'মধ্যরাত',
                noon: 'মধ্যাহ্ন',
                morning: 'সকাল',
                afternoon: 'বিকাল',
                evening: 'সন্ধ্যা',
                night: 'রাত',
              },
              abbreviated: {
                am: 'পূর্বাহ্ন',
                pm: 'অপরাহ্ন',
                midnight: 'মধ্যরাত',
                noon: 'মধ্যাহ্ন',
                morning: 'সকাল',
                afternoon: 'বিকাল',
                evening: 'সন্ধ্যা',
                night: 'রাত',
              },
              wide: {
                am: 'পূর্বাহ্ন',
                pm: 'অপরাহ্ন',
                midnight: 'মধ্যরাত',
                noon: 'মধ্যাহ্ন',
                morning: 'সকাল',
                afternoon: 'বিকাল',
                evening: 'সন্ধ্যা',
                night: 'রাত',
              },
            },
            defaultFormattingWidth: 'wide',
          }),
        };
        exports.default = _default;
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/match/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchFn/index.js',
            ),
          ),
          _default = {
            ordinalNumber: (0,
            _interopRequireDefault(
              __webpack_require__(
                '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchPatternFn/index.js',
              ),
            ).default)({
              matchPattern: /^(\d+)(ম|য়|র্থ|ষ্ঠ|শে|ই|তম)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(খ্রিঃপূঃ|খ্রিঃ)/i,
                abbreviated: /^(খ্রিঃপূর্ব|খ্রিঃ)/i,
                wide: /^(খ্রিস্টপূর্ব|খ্রিস্টাব্দ)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^খ্রিঃপূঃ/i, /^খ্রিঃ/i],
                abbreviated: [/^খ্রিঃপূর্ব/i, /^খ্রিঃ/i],
                wide: [/^খ্রিস্টপূর্ব/i, /^খ্রিস্টাব্দ/i],
              },
              defaultParseWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[১২৩৪]/i,
                abbreviated: /^[১২৩৪]ত্রৈ/i,
                wide: /^[১২৩৪](ম|য়|র্থ)? ত্রৈমাসিক/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/১/i, /২/i, /৩/i, /৪/i] },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^(জানু|ফেব্রু|মার্চ|এপ্রিল|মে|জুন|জুলাই|আগস্ট|সেপ্ট|অক্টো|নভে|ডিসে)/i,
                abbreviated: /^(জানু|ফেব্রু|মার্চ|এপ্রিল|মে|জুন|জুলাই|আগস্ট|সেপ্ট|অক্টো|নভে|ডিসে)/i,
                wide: /^(জানুয়ারি|ফেব্রুয়ারি|মার্চ|এপ্রিল|মে|জুন|জুলাই|আগস্ট|সেপ্টেম্বর|অক্টোবর|নভেম্বর|ডিসেম্বর)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: [
                  /^জানু/i,
                  /^ফেব্রু/i,
                  /^মার্চ/i,
                  /^এপ্রিল/i,
                  /^মে/i,
                  /^জুন/i,
                  /^জুলাই/i,
                  /^আগস্ট/i,
                  /^সেপ্ট/i,
                  /^অক্টো/i,
                  /^নভে/i,
                  /^ডিসে/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(র|সো|ম|বু|বৃ|শু|শ)+/i,
                short: /^(রবি|সোম|মঙ্গল|বুধ|বৃহ|শুক্র|শনি)+/i,
                abbreviated: /^(রবি|সোম|মঙ্গল|বুধ|বৃহ|শুক্র|শনি)+/i,
                wide: /^(রবিবার|সোমবার|মঙ্গলবার|বুধবার|বৃহস্পতিবার |শুক্রবার|শনিবার)+/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^র/i, /^সো/i, /^ম/i, /^বু/i, /^বৃ/i, /^শু/i, /^শ/i],
                short: [/^রবি/i, /^সোম/i, /^মঙ্গল/i, /^বুধ/i, /^বৃহ/i, /^শুক্র/i, /^শনি/i],
                abbreviated: [/^রবি/i, /^সোম/i, /^মঙ্গল/i, /^বুধ/i, /^বৃহ/i, /^শুক্র/i, /^শনি/i],
                wide: [
                  /^রবিবার/i,
                  /^সোমবার/i,
                  /^মঙ্গলবার/i,
                  /^বুধবার/i,
                  /^বৃহস্পতিবার /i,
                  /^শুক্রবার/i,
                  /^শনিবার/i,
                ],
              },
              defaultParseWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(পূ|অপ|মধ্যরাত|মধ্যাহ্ন|সকাল|বিকাল|সন্ধ্যা|রাত)/i,
                abbreviated: /^(পূর্বাহ্ন|অপরাহ্ন|মধ্যরাত|মধ্যাহ্ন|সকাল|বিকাল|সন্ধ্যা|রাত)/i,
                wide: /^(পূর্বাহ্ন|অপরাহ্ন|মধ্যরাত|মধ্যাহ্ন|সকাল|বিকাল|সন্ধ্যা|রাত)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: {
                  am: /^পূ/i,
                  pm: /^অপ/i,
                  midnight: /^মধ্যরাত/i,
                  noon: /^মধ্যাহ্ন/i,
                  morning: /সকাল/i,
                  afternoon: /বিকাল/i,
                  evening: /সন্ধ্যা/i,
                  night: /রাত/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/index.js': (
      module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      var _interopRequireDefault = __webpack_require__(
        '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
      ).default;
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
      var _index = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'bn',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 0, firstWeekContainsDate: 1 },
        };
      (exports.default = _default), (module.exports = exports.default);
    },
  },
]);
