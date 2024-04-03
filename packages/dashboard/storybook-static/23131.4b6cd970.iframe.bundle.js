(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [23131, 45585, 69107, 95565, 79489, 27789, 49704, 65098, 20306, 70358],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'بىر سىكۇنت ئىچىدە', other: 'سىكۇنت ئىچىدە {{count}}' },
            xSeconds: { one: 'بىر سىكۇنت', other: 'سىكۇنت {{count}}' },
            halfAMinute: 'يىرىم مىنۇت',
            lessThanXMinutes: { one: 'بىر مىنۇت ئىچىدە', other: 'مىنۇت ئىچىدە {{count}}' },
            xMinutes: { one: 'بىر مىنۇت', other: 'مىنۇت {{count}}' },
            aboutXHours: { one: 'تەخمىنەن بىر سائەت', other: 'سائەت {{count}} تەخمىنەن' },
            xHours: { one: 'بىر سائەت', other: 'سائەت {{count}}' },
            xDays: { one: 'بىر كۈن', other: 'كۈن {{count}}' },
            aboutXWeeks: { one: 'تەخمىنەن بىرھەپتە', other: 'ھەپتە {{count}} تەخمىنەن' },
            xWeeks: { one: 'بىرھەپتە', other: 'ھەپتە {{count}}' },
            aboutXMonths: { one: 'تەخمىنەن بىر ئاي', other: 'ئاي {{count}} تەخمىنەن' },
            xMonths: { one: 'بىر ئاي', other: 'ئاي {{count}}' },
            aboutXYears: { one: 'تەخمىنەن بىر يىل', other: 'يىل {{count}} تەخمىنەن' },
            xYears: { one: 'بىر يىل', other: 'يىل {{count}}' },
            overXYears: { one: 'بىر يىلدىن ئارتۇق', other: 'يىلدىن ئارتۇق {{count}}' },
            almostXYears: { one: 'ئاساسەن بىر يىل', other: 'يىل {{count}} ئاساسەن' },
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
                    : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? result
                  : result + ' بولدى'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatLong/index.js':
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
                full: "{{date}} 'دە' {{time}}",
                long: "{{date}} 'دە' {{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'ئ‍ۆتكەن' eeee 'دە' p",
            yesterday: "'تۈنۈگۈن دە' p",
            today: "'بۈگۈن دە' p",
            tomorrow: "'ئەتە دە' p",
            nextWeek: "eeee 'دە' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/match/index.js':
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
              matchPattern: /^(\d+)(th|st|nd|rd)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: { narrow: /^(ب|ك)/i, wide: /^(مىيلادىدىن بۇرۇن|مىيلادىدىن كىيىن)/i },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^بۇرۇن/i, /^كىيىن/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^چ[1234]/i,
                wide: /^چارەك [1234]/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/1/i, /2/i, /3/i, /4/i] },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^[يفمئامئ‍ئاسۆند]/i,
                abbreviated:
                  /^(يانۋار|فېۋىرال|مارت|ئاپرىل|ماي|ئىيۇن|ئىيول|ئاۋغۇست|سىنتەبىر|ئۆكتەبىر|نويابىر|دىكابىر)/i,
                wide: /^(يانۋار|فېۋىرال|مارت|ئاپرىل|ماي|ئىيۇن|ئىيول|ئاۋغۇست|سىنتەبىر|ئۆكتەبىر|نويابىر|دىكابىر)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^ي/i,
                  /^ف/i,
                  /^م/i,
                  /^ا/i,
                  /^م/i,
                  /^ى‍/i,
                  /^ى‍/i,
                  /^ا‍/i,
                  /^س/i,
                  /^ۆ/i,
                  /^ن/i,
                  /^د/i,
                ],
                any: [
                  /^يان/i,
                  /^فېۋ/i,
                  /^مار/i,
                  /^ئاپ/i,
                  /^ماي/i,
                  /^ئىيۇن/i,
                  /^ئىيول/i,
                  /^ئاۋ/i,
                  /^سىن/i,
                  /^ئۆك/i,
                  /^نوي/i,
                  /^دىك/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[دسچپجشي]/i,
                short: /^(يە|دۈ|سە|چا|پە|جۈ|شە)/i,
                abbreviated: /^(يە|دۈ|سە|چا|پە|جۈ|شە)/i,
                wide: /^(يەكشەنبە|دۈشەنبە|سەيشەنبە|چارشەنبە|پەيشەنبە|جۈمە|شەنبە)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^ي/i, /^د/i, /^س/i, /^چ/i, /^پ/i, /^ج/i, /^ش/i],
                any: [/^ي/i, /^د/i, /^س/i, /^چ/i, /^پ/i, /^ج/i, /^ش/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ئە|چ|ك|چ|(دە|ئەتىگەن) ( ئە‍|چۈشتىن كىيىن|ئاخشىم|كىچە))/i,
                any: /^(ئە|چ|ك|چ|(دە|ئەتىگەن) ( ئە‍|چۈشتىن كىيىن|ئاخشىم|كىچە))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^ئە/i,
                  pm: /^چ/i,
                  midnight: /^ك/i,
                  noon: /^چ/i,
                  morning: /ئەتىگەن/i,
                  afternoon: /چۈشتىن كىيىن/i,
                  evening: /ئاخشىم/i,
                  night: /كىچە/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'ug',
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
