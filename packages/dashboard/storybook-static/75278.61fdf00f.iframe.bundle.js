(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [75278, 45585, 69107, 95565, 79489, 13764, 65109, 79127, 9463, 67641],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        function declension(scheme, count) {
          if (void 0 !== scheme.one && 1 === count) return scheme.one;
          var rem10 = count % 10,
            rem100 = count % 100;
          return 1 === rem10 && 11 !== rem100
            ? scheme.singularNominative.replace('{{count}}', String(count))
            : rem10 >= 2 && rem10 <= 4 && (rem100 < 10 || rem100 > 20)
              ? scheme.singularGenitive.replace('{{count}}', String(count))
              : scheme.pluralGenitive.replace('{{count}}', String(count));
        }
        function buildLocalizeTokenFn(scheme) {
          return function (count, options) {
            return null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? scheme.future
                  ? declension(scheme.future, count)
                  : 'через ' + declension(scheme.regular, count)
                : scheme.past
                  ? declension(scheme.past, count)
                  : declension(scheme.regular, count) + ' назад'
              : declension(scheme.regular, count);
          };
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: buildLocalizeTokenFn({
              regular: {
                one: 'меньше секунды',
                singularNominative: 'меньше {{count}} секунды',
                singularGenitive: 'меньше {{count}} секунд',
                pluralGenitive: 'меньше {{count}} секунд',
              },
              future: {
                one: 'меньше, чем через секунду',
                singularNominative: 'меньше, чем через {{count}} секунду',
                singularGenitive: 'меньше, чем через {{count}} секунды',
                pluralGenitive: 'меньше, чем через {{count}} секунд',
              },
            }),
            xSeconds: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} секунда',
                singularGenitive: '{{count}} секунды',
                pluralGenitive: '{{count}} секунд',
              },
              past: {
                singularNominative: '{{count}} секунду назад',
                singularGenitive: '{{count}} секунды назад',
                pluralGenitive: '{{count}} секунд назад',
              },
              future: {
                singularNominative: 'через {{count}} секунду',
                singularGenitive: 'через {{count}} секунды',
                pluralGenitive: 'через {{count}} секунд',
              },
            }),
            halfAMinute: function halfAMinute(_count, options) {
              return null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'через полминуты'
                  : 'полминуты назад'
                : 'полминуты';
            },
            lessThanXMinutes: buildLocalizeTokenFn({
              regular: {
                one: 'меньше минуты',
                singularNominative: 'меньше {{count}} минуты',
                singularGenitive: 'меньше {{count}} минут',
                pluralGenitive: 'меньше {{count}} минут',
              },
              future: {
                one: 'меньше, чем через минуту',
                singularNominative: 'меньше, чем через {{count}} минуту',
                singularGenitive: 'меньше, чем через {{count}} минуты',
                pluralGenitive: 'меньше, чем через {{count}} минут',
              },
            }),
            xMinutes: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} минута',
                singularGenitive: '{{count}} минуты',
                pluralGenitive: '{{count}} минут',
              },
              past: {
                singularNominative: '{{count}} минуту назад',
                singularGenitive: '{{count}} минуты назад',
                pluralGenitive: '{{count}} минут назад',
              },
              future: {
                singularNominative: 'через {{count}} минуту',
                singularGenitive: 'через {{count}} минуты',
                pluralGenitive: 'через {{count}} минут',
              },
            }),
            aboutXHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} часа',
                singularGenitive: 'около {{count}} часов',
                pluralGenitive: 'около {{count}} часов',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} час',
                singularGenitive: 'приблизительно через {{count}} часа',
                pluralGenitive: 'приблизительно через {{count}} часов',
              },
            }),
            xHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} час',
                singularGenitive: '{{count}} часа',
                pluralGenitive: '{{count}} часов',
              },
            }),
            xDays: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} день',
                singularGenitive: '{{count}} дня',
                pluralGenitive: '{{count}} дней',
              },
            }),
            aboutXWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} недели',
                singularGenitive: 'около {{count}} недель',
                pluralGenitive: 'около {{count}} недель',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} неделю',
                singularGenitive: 'приблизительно через {{count}} недели',
                pluralGenitive: 'приблизительно через {{count}} недель',
              },
            }),
            xWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} неделя',
                singularGenitive: '{{count}} недели',
                pluralGenitive: '{{count}} недель',
              },
            }),
            aboutXMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} месяца',
                singularGenitive: 'около {{count}} месяцев',
                pluralGenitive: 'около {{count}} месяцев',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} месяц',
                singularGenitive: 'приблизительно через {{count}} месяца',
                pluralGenitive: 'приблизительно через {{count}} месяцев',
              },
            }),
            xMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} месяц',
                singularGenitive: '{{count}} месяца',
                pluralGenitive: '{{count}} месяцев',
              },
            }),
            aboutXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} года',
                singularGenitive: 'около {{count}} лет',
                pluralGenitive: 'около {{count}} лет',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} год',
                singularGenitive: 'приблизительно через {{count}} года',
                pluralGenitive: 'приблизительно через {{count}} лет',
              },
            }),
            xYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} год',
                singularGenitive: '{{count}} года',
                pluralGenitive: '{{count}} лет',
              },
            }),
            overXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'больше {{count}} года',
                singularGenitive: 'больше {{count}} лет',
                pluralGenitive: 'больше {{count}} лет',
              },
              future: {
                singularNominative: 'больше, чем через {{count}} год',
                singularGenitive: 'больше, чем через {{count}} года',
                pluralGenitive: 'больше, чем через {{count}} лет',
              },
            }),
            almostXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'почти {{count}} год',
                singularGenitive: 'почти {{count}} года',
                pluralGenitive: 'почти {{count}} лет',
              },
              future: {
                singularNominative: 'почти через {{count}} год',
                singularGenitive: 'почти через {{count}} года',
                pluralGenitive: 'почти через {{count}} лет',
              },
            }),
          },
          _default = function formatDistance(token, count, options) {
            return formatDistanceLocale[token](count, options);
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatLong/index.js':
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
                full: "EEEE, d MMMM y 'г.'",
                long: "d MMMM y 'г.'",
                medium: "d MMM y 'г.'",
                short: 'dd.MM.y',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'H:mm:ss zzzz',
                long: 'H:mm:ss z',
                medium: 'H:mm:ss',
                short: 'H:mm',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: { any: '{{date}}, {{time}}' },
              defaultWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatRelative/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/isSameUTCWeek/index.js',
            ),
          ),
          accusativeWeekdays = [
            'воскресенье',
            'понедельник',
            'вторник',
            'среду',
            'четверг',
            'пятницу',
            'субботу',
          ];
        function thisWeek(day) {
          var weekday = accusativeWeekdays[day];
          return 2 === day ? "'во " + weekday + " в' p" : "'в " + weekday + " в' p";
        }
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date, baseDate, options) {
              var day = date.getUTCDay();
              return (0, _index.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function _lastWeek(day) {
                    var weekday = accusativeWeekdays[day];
                    switch (day) {
                      case 0:
                        return "'в прошлое " + weekday + " в' p";
                      case 1:
                      case 2:
                      case 4:
                        return "'в прошлый " + weekday + " в' p";
                      case 3:
                      case 5:
                      case 6:
                        return "'в прошлую " + weekday + " в' p";
                    }
                  })(day);
            },
            yesterday: "'вчера в' p",
            today: "'сегодня в' p",
            tomorrow: "'завтра в' p",
            nextWeek: function nextWeek(date, baseDate, options) {
              var day = date.getUTCDay();
              return (0, _index.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function _nextWeek(day) {
                    var weekday = accusativeWeekdays[day];
                    switch (day) {
                      case 0:
                        return "'в следующее " + weekday + " в' p";
                      case 1:
                      case 2:
                      case 4:
                        return "'в следующий " + weekday + " в' p";
                      case 3:
                      case 5:
                      case 6:
                        return "'в следующую " + weekday + " в' p";
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/localize/index.js':
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
              var number = Number(dirtyNumber),
                unit = null == options ? void 0 : options.unit;
              return (
                number +
                ('date' === unit
                  ? '-е'
                  : 'week' === unit || 'minute' === unit || 'second' === unit
                    ? '-я'
                    : '-й')
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['до н.э.', 'н.э.'],
                abbreviated: ['до н. э.', 'н. э.'],
                wide: ['до нашей эры', 'нашей эры'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-й кв.', '2-й кв.', '3-й кв.', '4-й кв.'],
                wide: ['1-й квартал', '2-й квартал', '3-й квартал', '4-й квартал'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['Я', 'Ф', 'М', 'А', 'М', 'И', 'И', 'А', 'С', 'О', 'Н', 'Д'],
                abbreviated: [
                  'янв.',
                  'фев.',
                  'март',
                  'апр.',
                  'май',
                  'июнь',
                  'июль',
                  'авг.',
                  'сент.',
                  'окт.',
                  'нояб.',
                  'дек.',
                ],
                wide: [
                  'январь',
                  'февраль',
                  'март',
                  'апрель',
                  'май',
                  'июнь',
                  'июль',
                  'август',
                  'сентябрь',
                  'октябрь',
                  'ноябрь',
                  'декабрь',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['Я', 'Ф', 'М', 'А', 'М', 'И', 'И', 'А', 'С', 'О', 'Н', 'Д'],
                abbreviated: [
                  'янв.',
                  'фев.',
                  'мар.',
                  'апр.',
                  'мая',
                  'июн.',
                  'июл.',
                  'авг.',
                  'сент.',
                  'окт.',
                  'нояб.',
                  'дек.',
                ],
                wide: [
                  'января',
                  'февраля',
                  'марта',
                  'апреля',
                  'мая',
                  'июня',
                  'июля',
                  'августа',
                  'сентября',
                  'октября',
                  'ноября',
                  'декабря',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['В', 'П', 'В', 'С', 'Ч', 'П', 'С'],
                short: ['вс', 'пн', 'вт', 'ср', 'чт', 'пт', 'сб'],
                abbreviated: ['вск', 'пнд', 'втр', 'срд', 'чтв', 'птн', 'суб'],
                wide: [
                  'воскресенье',
                  'понедельник',
                  'вторник',
                  'среда',
                  'четверг',
                  'пятница',
                  'суббота',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утро',
                  afternoon: 'день',
                  evening: 'веч.',
                  night: 'ночь',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утро',
                  afternoon: 'день',
                  evening: 'веч.',
                  night: 'ночь',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полночь',
                  noon: 'полдень',
                  morning: 'утро',
                  afternoon: 'день',
                  evening: 'вечер',
                  night: 'ночь',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утра',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночи',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утра',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночи',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полночь',
                  noon: 'полдень',
                  morning: 'утра',
                  afternoon: 'дня',
                  evening: 'вечера',
                  night: 'ночи',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/match/index.js':
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
              matchPattern: /^(\d+)(-?(е|я|й|ое|ье|ая|ья|ый|ой|ий|ый))?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^((до )?н\.?\s?э\.?)/i,
                abbreviated: /^((до )?н\.?\s?э\.?)/i,
                wide: /^(до нашей эры|нашей эры|наша эра)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^д/i, /^н/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234](-?[ыои]?й?)? кв.?/i,
                wide: /^[1234](-?[ыои]?й?)? квартал/i,
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
                narrow: /^[яфмаисонд]/i,
                abbreviated:
                  /^(янв|фев|март?|апр|ма[йя]|июн[ья]?|июл[ья]?|авг|сент?|окт|нояб?|дек)\.?/i,
                wide: /^(январ[ья]|феврал[ья]|марта?|апрел[ья]|ма[йя]|июн[ья]|июл[ья]|августа?|сентябр[ья]|октябр[ья]|октябр[ья]|ноябр[ья]|декабр[ья])/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^я/i,
                  /^ф/i,
                  /^м/i,
                  /^а/i,
                  /^м/i,
                  /^и/i,
                  /^и/i,
                  /^а/i,
                  /^с/i,
                  /^о/i,
                  /^н/i,
                  /^я/i,
                ],
                any: [
                  /^я/i,
                  /^ф/i,
                  /^мар/i,
                  /^ап/i,
                  /^ма[йя]/i,
                  /^июн/i,
                  /^июл/i,
                  /^ав/i,
                  /^с/i,
                  /^о/i,
                  /^н/i,
                  /^д/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[впсч]/i,
                short: /^(вс|во|пн|по|вт|ср|чт|че|пт|пя|сб|су)\.?/i,
                abbreviated: /^(вск|вос|пнд|пон|втр|вто|срд|сре|чтв|чет|птн|пят|суб).?/i,
                wide: /^(воскресень[ея]|понедельника?|вторника?|сред[аы]|четверга?|пятниц[аы]|суббот[аы])/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^в/i, /^п/i, /^в/i, /^с/i, /^ч/i, /^п/i, /^с/i],
                any: [/^в[ос]/i, /^п[он]/i, /^в/i, /^ср/i, /^ч/i, /^п[ят]/i, /^с[уб]/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^([дп]п|полн\.?|полд\.?|утр[оа]|день|дня|веч\.?|ноч[ьи])/i,
                abbreviated: /^([дп]п|полн\.?|полд\.?|утр[оа]|день|дня|веч\.?|ноч[ьи])/i,
                wide: /^([дп]п|полночь|полдень|утр[оа]|день|дня|вечера?|ноч[ьи])/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: {
                  am: /^дп/i,
                  pm: /^пп/i,
                  midnight: /^полн/i,
                  noon: /^полд/i,
                  morning: /^у/i,
                  afternoon: /^д[ен]/i,
                  evening: /^в/i,
                  night: /^н/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'ru',
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
