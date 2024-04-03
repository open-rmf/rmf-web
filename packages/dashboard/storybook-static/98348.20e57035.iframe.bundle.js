(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [98348, 45585, 69107, 95565, 79489, 55418, 87643, 18105, 15409, 94063],
  {
    '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js':
      (module) => {
        (module.exports = function _interopRequireDefault(obj) {
          return obj && obj.__esModule ? obj : { default: obj };
        }),
          (module.exports.__esModule = !0),
          (module.exports.default = module.exports);
      },
    '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/typeof.js':
      (module) => {
        function _typeof(o) {
          return (
            (module.exports = _typeof =
              'function' == typeof Symbol && 'symbol' == typeof Symbol.iterator
                ? function (o) {
                    return typeof o;
                  }
                : function (o) {
                    return o &&
                      'function' == typeof Symbol &&
                      o.constructor === Symbol &&
                      o !== Symbol.prototype
                      ? 'symbol'
                      : typeof o;
                  }),
            (module.exports.__esModule = !0),
            (module.exports.default = module.exports),
            _typeof(o)
          );
        }
        (module.exports = _typeof),
          (module.exports.__esModule = !0),
          (module.exports.default = module.exports);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/defaultOptions/index.js': (
      __unused_webpack_module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.getDefaultOptions = function getDefaultOptions() {
          return defaultOptions;
        }),
        (exports.setDefaultOptions = function setDefaultOptions(newOptions) {
          defaultOptions = newOptions;
        });
      var defaultOptions = {};
    },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/isSameUTCWeek/index.js': (
      module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      var _interopRequireDefault = __webpack_require__(
        '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
      ).default;
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.default = function isSameUTCWeek(dirtyDateLeft, dirtyDateRight, options) {
          (0, _index.default)(2, arguments);
          var dateLeftStartOfWeek = (0, _index2.default)(dirtyDateLeft, options),
            dateRightStartOfWeek = (0, _index2.default)(dirtyDateRight, options);
          return dateLeftStartOfWeek.getTime() === dateRightStartOfWeek.getTime();
        });
      var _index = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/requiredArgs/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/startOfUTCWeek/index.js',
          ),
        );
      module.exports = exports.default;
    },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/requiredArgs/index.js': (
      module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.default = function requiredArgs(required, args) {
          if (args.length < required)
            throw new TypeError(
              required +
                ' argument' +
                (required > 1 ? 's' : '') +
                ' required, but only ' +
                args.length +
                ' present',
            );
        }),
        (module.exports = exports.default);
    },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/startOfUTCWeek/index.js': (
      module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      var _interopRequireDefault = __webpack_require__(
        '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
      ).default;
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.default = function startOfUTCWeek(dirtyDate, options) {
          var _ref,
            _ref2,
            _ref3,
            _options$weekStartsOn,
            _options$locale,
            _options$locale$optio,
            _defaultOptions$local,
            _defaultOptions$local2;
          (0, _index2.default)(1, arguments);
          var defaultOptions = (0, _index4.getDefaultOptions)(),
            weekStartsOn = (0, _index3.default)(
              null !==
                (_ref =
                  null !==
                    (_ref2 =
                      null !==
                        (_ref3 =
                          null !==
                            (_options$weekStartsOn =
                              null == options ? void 0 : options.weekStartsOn) &&
                          void 0 !== _options$weekStartsOn
                            ? _options$weekStartsOn
                            : null == options ||
                                null === (_options$locale = options.locale) ||
                                void 0 === _options$locale ||
                                null === (_options$locale$optio = _options$locale.options) ||
                                void 0 === _options$locale$optio
                              ? void 0
                              : _options$locale$optio.weekStartsOn) && void 0 !== _ref3
                        ? _ref3
                        : defaultOptions.weekStartsOn) && void 0 !== _ref2
                    ? _ref2
                    : null === (_defaultOptions$local = defaultOptions.locale) ||
                        void 0 === _defaultOptions$local ||
                        null === (_defaultOptions$local2 = _defaultOptions$local.options) ||
                        void 0 === _defaultOptions$local2
                      ? void 0
                      : _defaultOptions$local2.weekStartsOn) && void 0 !== _ref
                ? _ref
                : 0,
            );
          if (!(weekStartsOn >= 0 && weekStartsOn <= 6))
            throw new RangeError('weekStartsOn must be between 0 and 6 inclusively');
          var date = (0, _index.default)(dirtyDate),
            day = date.getUTCDay(),
            diff = (day < weekStartsOn ? 7 : 0) + day - weekStartsOn;
          return date.setUTCDate(date.getUTCDate() - diff), date.setUTCHours(0, 0, 0, 0), date;
        });
      var _index = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/toDate/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/requiredArgs/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/toInteger/index.js',
          ),
        ),
        _index4 = __webpack_require__(
          '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/defaultOptions/index.js',
        );
      module.exports = exports.default;
    },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/toInteger/index.js': (
      module,
      exports,
    ) => {
      'use strict';
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.default = function toInteger(dirtyNumber) {
          if (null === dirtyNumber || !0 === dirtyNumber || !1 === dirtyNumber) return NaN;
          var number = Number(dirtyNumber);
          if (isNaN(number)) return number;
          return number < 0 ? Math.ceil(number) : Math.floor(number);
        }),
        (module.exports = exports.default);
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'по-малко от секунда',
              other: 'по-малко от {{count}} секунди',
            },
            xSeconds: { one: '1 секунда', other: '{{count}} секунди' },
            halfAMinute: 'половин минута',
            lessThanXMinutes: { one: 'по-малко от минута', other: 'по-малко от {{count}} минути' },
            xMinutes: { one: '1 минута', other: '{{count}} минути' },
            aboutXHours: { one: 'около час', other: 'около {{count}} часа' },
            xHours: { one: '1 час', other: '{{count}} часа' },
            xDays: { one: '1 ден', other: '{{count}} дни' },
            aboutXWeeks: { one: 'около седмица', other: 'около {{count}} седмици' },
            xWeeks: { one: '1 седмица', other: '{{count}} седмици' },
            aboutXMonths: { one: 'около месец', other: 'около {{count}} месеца' },
            xMonths: { one: '1 месец', other: '{{count}} месеца' },
            aboutXYears: { one: 'около година', other: 'около {{count}} години' },
            xYears: { one: '1 година', other: '{{count}} години' },
            overXYears: { one: 'над година', other: 'над {{count}} години' },
            almostXYears: { one: 'почти година', other: 'почти {{count}} години' },
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
                  ? 'след ' + result
                  : 'преди ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatLong/index.js':
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
                full: 'EEEE, dd MMMM yyyy',
                long: 'dd MMMM yyyy',
                medium: 'dd MMM yyyy',
                short: 'dd/MM/yyyy',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'HH:mm:ss zzzz',
                long: 'HH:mm:ss z',
                medium: 'HH:mm:ss',
                short: 'H:mm',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: { any: '{{date}} {{time}}' },
              defaultWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatRelative/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/toDate/index.js',
            ),
          ),
          _index2 = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/isSameUTCWeek/index.js',
            ),
          ),
          weekdays = ['неделя', 'понеделник', 'вторник', 'сряда', 'четвъртък', 'петък', 'събота'];
        function thisWeek(day) {
          var weekday = weekdays[day];
          return 2 === day ? "'във " + weekday + " в' p" : "'в " + weekday + " в' p";
        }
        var formatRelativeLocale = {
            lastWeek: function lastWeekFormatToken(dirtyDate, baseDate, options) {
              var date = (0, _index.default)(dirtyDate),
                day = date.getUTCDay();
              return (0, _index2.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function lastWeek(day) {
                    var weekday = weekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 6:
                        return "'миналата " + weekday + " в' p";
                      case 1:
                      case 2:
                      case 4:
                      case 5:
                        return "'миналия " + weekday + " в' p";
                    }
                  })(day);
            },
            yesterday: "'вчера в' p",
            today: "'днес в' p",
            tomorrow: "'утре в' p",
            nextWeek: function nextWeekFormatToken(dirtyDate, baseDate, options) {
              var date = (0, _index.default)(dirtyDate),
                day = date.getUTCDay();
              return (0, _index2.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function nextWeek(day) {
                    var weekday = weekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 6:
                        return "'следващата " + weekday + " в' p";
                      case 1:
                      case 2:
                      case 4:
                      case 5:
                        return "'следващия " + weekday + " в' p";
                    }
                  })(day);
            },
            other: 'P',
          },
          _default = function formatRelative(token, date, baseDate, options) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date, baseDate, options) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/localize/index.js':
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
        );
        function numberWithSuffix(number, unit, masculine, feminine, neuter) {
          var suffix = (function isNeuter(unit) {
            return 'quarter' === unit;
          })(unit)
            ? neuter
            : (function isFeminine(unit) {
                  return (
                    'year' === unit || 'week' === unit || 'minute' === unit || 'second' === unit
                  );
                })(unit)
              ? feminine
              : masculine;
          return number + '-' + suffix;
        }
        var _default = {
          ordinalNumber: function ordinalNumber(dirtyNumber, options) {
            var number = Number(dirtyNumber),
              unit = null == options ? void 0 : options.unit;
            if (0 === number) return numberWithSuffix(0, unit, 'ев', 'ева', 'ево');
            if (number % 1e3 == 0) return numberWithSuffix(number, unit, 'ен', 'на', 'но');
            if (number % 100 == 0) return numberWithSuffix(number, unit, 'тен', 'тна', 'тно');
            var rem100 = number % 100;
            if (rem100 > 20 || rem100 < 10)
              switch (rem100 % 10) {
                case 1:
                  return numberWithSuffix(number, unit, 'ви', 'ва', 'во');
                case 2:
                  return numberWithSuffix(number, unit, 'ри', 'ра', 'ро');
                case 7:
                case 8:
                  return numberWithSuffix(number, unit, 'ми', 'ма', 'мо');
              }
            return numberWithSuffix(number, unit, 'ти', 'та', 'то');
          },
          era: (0, _index.default)({
            values: {
              narrow: ['пр.н.е.', 'н.е.'],
              abbreviated: ['преди н. е.', 'н. е.'],
              wide: ['преди новата ера', 'новата ера'],
            },
            defaultWidth: 'wide',
          }),
          quarter: (0, _index.default)({
            values: {
              narrow: ['1', '2', '3', '4'],
              abbreviated: ['1-во тримес.', '2-ро тримес.', '3-то тримес.', '4-то тримес.'],
              wide: ['1-во тримесечие', '2-ро тримесечие', '3-то тримесечие', '4-то тримесечие'],
            },
            defaultWidth: 'wide',
            argumentCallback: function argumentCallback(quarter) {
              return quarter - 1;
            },
          }),
          month: (0, _index.default)({
            values: {
              abbreviated: [
                'яну',
                'фев',
                'мар',
                'апр',
                'май',
                'юни',
                'юли',
                'авг',
                'сеп',
                'окт',
                'ное',
                'дек',
              ],
              wide: [
                'януари',
                'февруари',
                'март',
                'април',
                'май',
                'юни',
                'юли',
                'август',
                'септември',
                'октомври',
                'ноември',
                'декември',
              ],
            },
            defaultWidth: 'wide',
          }),
          day: (0, _index.default)({
            values: {
              narrow: ['Н', 'П', 'В', 'С', 'Ч', 'П', 'С'],
              short: ['нд', 'пн', 'вт', 'ср', 'чт', 'пт', 'сб'],
              abbreviated: ['нед', 'пон', 'вто', 'сря', 'чет', 'пет', 'съб'],
              wide: ['неделя', 'понеделник', 'вторник', 'сряда', 'четвъртък', 'петък', 'събота'],
            },
            defaultWidth: 'wide',
          }),
          dayPeriod: (0, _index.default)({
            values: {
              wide: {
                am: 'преди обяд',
                pm: 'след обяд',
                midnight: 'в полунощ',
                noon: 'на обяд',
                morning: 'сутринта',
                afternoon: 'следобед',
                evening: 'вечерта',
                night: 'през нощта',
              },
            },
            defaultWidth: 'wide',
          }),
        };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/match/index.js':
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
              matchPattern: /^(\d+)(-?[врмт][аи]|-?т?(ен|на)|-?(ев|ева))?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^((пр)?н\.?\s?е\.?)/i,
                abbreviated: /^((пр)?н\.?\s?е\.?)/i,
                wide: /^(преди новата ера|новата ера|нова ера)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^п/i, /^н/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234](-?[врт]?o?)? тримес.?/i,
                wide: /^[1234](-?[врт]?о?)? тримесечие/i,
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
                abbreviated: /^(яну|фев|мар|апр|май|юни|юли|авг|сеп|окт|ное|дек)/i,
                wide: /^(януари|февруари|март|април|май|юни|юли|август|септември|октомври|ноември|декември)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: [
                  /^я/i,
                  /^ф/i,
                  /^мар/i,
                  /^ап/i,
                  /^май/i,
                  /^юн/i,
                  /^юл/i,
                  /^ав/i,
                  /^се/i,
                  /^окт/i,
                  /^но/i,
                  /^де/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[нпвсч]/i,
                short: /^(нд|пн|вт|ср|чт|пт|сб)/i,
                abbreviated: /^(нед|пон|вто|сря|чет|пет|съб)/i,
                wide: /^(неделя|понеделник|вторник|сряда|четвъртък|петък|събота)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^н/i, /^п/i, /^в/i, /^с/i, /^ч/i, /^п/i, /^с/i],
                any: [/^н[ед]/i, /^п[он]/i, /^вт/i, /^ср/i, /^ч[ет]/i, /^п[ет]/i, /^с[ъб]/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: { any: /^(преди о|след о|в по|на о|през|веч|сут|следо)/i },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^преди о/i,
                  pm: /^след о/i,
                  midnight: /^в пол/i,
                  noon: /^на об/i,
                  morning: /^сут/i,
                  afternoon: /^следо/i,
                  evening: /^веч/i,
                  night: /^през н/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'bg',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 1, firstWeekContainsDate: 1 },
        };
      (exports.default = _default), (module.exports = exports.default);
    },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/toDate/index.js': (
      module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      var _interopRequireDefault = __webpack_require__(
        '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
      ).default;
      Object.defineProperty(exports, '__esModule', { value: !0 }),
        (exports.default = function toDate(argument) {
          (0, _index.default)(1, arguments);
          var argStr = Object.prototype.toString.call(argument);
          return argument instanceof Date ||
            ('object' === (0, _typeof2.default)(argument) && '[object Date]' === argStr)
            ? new Date(argument.getTime())
            : 'number' == typeof argument || '[object Number]' === argStr
              ? new Date(argument)
              : (('string' != typeof argument && '[object String]' !== argStr) ||
                  'undefined' == typeof console ||
                  (console.warn(
                    "Starting with v2.0.0-beta.1 date-fns doesn't accept strings as date arguments. Please use `parseISO` to parse strings. See: https://github.com/date-fns/date-fns/blob/master/docs/upgradeGuide.md#string-arguments",
                  ),
                  console.warn(new Error().stack)),
                new Date(NaN));
        });
      var _typeof2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/typeof.js',
          ),
        ),
        _index = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/requiredArgs/index.js',
          ),
        );
      module.exports = exports.default;
    },
  },
]);
