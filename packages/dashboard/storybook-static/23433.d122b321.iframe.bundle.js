(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [23433, 45585, 69107, 95565, 79489, 85987, 3598, 1052, 61284, 81380],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
          lessThanXSeconds: {
            regular: {
              one: '1 секундтан аз',
              singularNominative: '{{count}} секундтан аз',
              singularGenitive: '{{count}} секундтан аз',
              pluralGenitive: '{{count}} секундтан аз',
            },
            future: {
              one: 'бір секундтан кейін',
              singularNominative: '{{count}} секундтан кейін',
              singularGenitive: '{{count}} секундтан кейін',
              pluralGenitive: '{{count}} секундтан кейін',
            },
          },
          xSeconds: {
            regular: {
              singularNominative: '{{count}} секунд',
              singularGenitive: '{{count}} секунд',
              pluralGenitive: '{{count}} секунд',
            },
            past: {
              singularNominative: '{{count}} секунд бұрын',
              singularGenitive: '{{count}} секунд бұрын',
              pluralGenitive: '{{count}} секунд бұрын',
            },
            future: {
              singularNominative: '{{count}} секундтан кейін',
              singularGenitive: '{{count}} секундтан кейін',
              pluralGenitive: '{{count}} секундтан кейін',
            },
          },
          halfAMinute: function halfAMinute(options) {
            return null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? 'жарты минут ішінде'
                : 'жарты минут бұрын'
              : 'жарты минут';
          },
          lessThanXMinutes: {
            regular: {
              one: '1 минуттан аз',
              singularNominative: '{{count}} минуттан аз',
              singularGenitive: '{{count}} минуттан аз',
              pluralGenitive: '{{count}} минуттан аз',
            },
            future: {
              one: 'минуттан кем ',
              singularNominative: '{{count}} минуттан кем',
              singularGenitive: '{{count}} минуттан кем',
              pluralGenitive: '{{count}} минуттан кем',
            },
          },
          xMinutes: {
            regular: {
              singularNominative: '{{count}} минут',
              singularGenitive: '{{count}} минут',
              pluralGenitive: '{{count}} минут',
            },
            past: {
              singularNominative: '{{count}} минут бұрын',
              singularGenitive: '{{count}} минут бұрын',
              pluralGenitive: '{{count}} минут бұрын',
            },
            future: {
              singularNominative: '{{count}} минуттан кейін',
              singularGenitive: '{{count}} минуттан кейін',
              pluralGenitive: '{{count}} минуттан кейін',
            },
          },
          aboutXHours: {
            regular: {
              singularNominative: 'шамамен {{count}} сағат',
              singularGenitive: 'шамамен {{count}} сағат',
              pluralGenitive: 'шамамен {{count}} сағат',
            },
            future: {
              singularNominative: 'шамамен {{count}} сағаттан кейін',
              singularGenitive: 'шамамен {{count}} сағаттан кейін',
              pluralGenitive: 'шамамен {{count}} сағаттан кейін',
            },
          },
          xHours: {
            regular: {
              singularNominative: '{{count}} сағат',
              singularGenitive: '{{count}} сағат',
              pluralGenitive: '{{count}} сағат',
            },
          },
          xDays: {
            regular: {
              singularNominative: '{{count}} күн',
              singularGenitive: '{{count}} күн',
              pluralGenitive: '{{count}} күн',
            },
            future: {
              singularNominative: '{{count}} күннен кейін',
              singularGenitive: '{{count}} күннен кейін',
              pluralGenitive: '{{count}} күннен кейін',
            },
          },
          aboutXWeeks: { type: 'weeks', one: 'шамамен 1 апта', other: 'шамамен {{count}} апта' },
          xWeeks: { type: 'weeks', one: '1 апта', other: '{{count}} апта' },
          aboutXMonths: {
            regular: {
              singularNominative: 'шамамен {{count}} ай',
              singularGenitive: 'шамамен {{count}} ай',
              pluralGenitive: 'шамамен {{count}} ай',
            },
            future: {
              singularNominative: 'шамамен {{count}} айдан кейін',
              singularGenitive: 'шамамен {{count}} айдан кейін',
              pluralGenitive: 'шамамен {{count}} айдан кейін',
            },
          },
          xMonths: {
            regular: {
              singularNominative: '{{count}} ай',
              singularGenitive: '{{count}} ай',
              pluralGenitive: '{{count}} ай',
            },
          },
          aboutXYears: {
            regular: {
              singularNominative: 'шамамен {{count}} жыл',
              singularGenitive: 'шамамен {{count}} жыл',
              pluralGenitive: 'шамамен {{count}} жыл',
            },
            future: {
              singularNominative: 'шамамен {{count}} жылдан кейін',
              singularGenitive: 'шамамен {{count}} жылдан кейін',
              pluralGenitive: 'шамамен {{count}} жылдан кейін',
            },
          },
          xYears: {
            regular: {
              singularNominative: '{{count}} жыл',
              singularGenitive: '{{count}} жыл',
              pluralGenitive: '{{count}} жыл',
            },
            future: {
              singularNominative: '{{count}} жылдан кейін',
              singularGenitive: '{{count}} жылдан кейін',
              pluralGenitive: '{{count}} жылдан кейін',
            },
          },
          overXYears: {
            regular: {
              singularNominative: '{{count}} жылдан астам',
              singularGenitive: '{{count}} жылдан астам',
              pluralGenitive: '{{count}} жылдан астам',
            },
            future: {
              singularNominative: '{{count}} жылдан астам',
              singularGenitive: '{{count}} жылдан астам',
              pluralGenitive: '{{count}} жылдан астам',
            },
          },
          almostXYears: {
            regular: {
              singularNominative: '{{count}} жылға жақын',
              singularGenitive: '{{count}} жылға жақын',
              pluralGenitive: '{{count}} жылға жақын',
            },
            future: {
              singularNominative: '{{count}} жылдан кейін',
              singularGenitive: '{{count}} жылдан кейін',
              pluralGenitive: '{{count}} жылдан кейін',
            },
          },
        };
        function declension(scheme, count) {
          if (scheme.one && 1 === count) return scheme.one;
          var rem10 = count % 10,
            rem100 = count % 100;
          return 1 === rem10 && 11 !== rem100
            ? scheme.singularNominative.replace('{{count}}', String(count))
            : rem10 >= 2 && rem10 <= 4 && (rem100 < 10 || rem100 > 20)
              ? scheme.singularGenitive.replace('{{count}}', String(count))
              : scheme.pluralGenitive.replace('{{count}}', String(count));
        }
        var _default = function formatDistance(token, count, options) {
          var tokenValue = formatDistanceLocale[token];
          return 'function' == typeof tokenValue
            ? tokenValue(options)
            : 'weeks' === tokenValue.type
              ? 1 === count
                ? tokenValue.one
                : tokenValue.other.replace('{{count}}', String(count))
              : null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? tokenValue.future
                    ? declension(tokenValue.future, count)
                    : declension(tokenValue.regular, count) + ' кейін'
                  : tokenValue.past
                    ? declension(tokenValue.past, count)
                    : declension(tokenValue.regular, count) + ' бұрын'
                : declension(tokenValue.regular, count);
        };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatLong/index.js':
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
                full: "EEEE, do MMMM y 'ж.'",
                long: "do MMMM y 'ж.'",
                medium: "d MMM y 'ж.'",
                short: 'dd.MM.yyyy',
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatRelative/index.js':
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
            'жексенбіде',
            'дүйсенбіде',
            'сейсенбіде',
            'сәрсенбіде',
            'бейсенбіде',
            'жұмада',
            'сенбіде',
          ];
        function thisWeek(day) {
          return "'" + accusativeWeekdays[day] + " сағат' p'-де'";
        }
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date, baseDate, options) {
              var day = date.getUTCDay();
              return (0, _index.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function _lastWeek(day) {
                    return "'өткен " + accusativeWeekdays[day] + " сағат' p'-де'";
                  })(day);
            },
            yesterday: "'кеше сағат' p'-де'",
            today: "'бүгін сағат' p'-де'",
            tomorrow: "'ертең сағат' p'-де'",
            nextWeek: function nextWeek(date, baseDate, options) {
              var day = date.getUTCDay();
              return (0, _index.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function _nextWeek(day) {
                    return "'келесі " + accusativeWeekdays[day] + " сағат' p'-де'";
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/localize/index.js':
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
          suffixes = {
            0: '-ші',
            1: '-ші',
            2: '-ші',
            3: '-ші',
            4: '-ші',
            5: '-ші',
            6: '-шы',
            7: '-ші',
            8: '-ші',
            9: '-шы',
            10: '-шы',
            20: '-шы',
            30: '-шы',
            40: '-шы',
            50: '-ші',
            60: '-шы',
            70: '-ші',
            80: '-ші',
            90: '-шы',
            100: '-ші',
          },
          _default = {
            ordinalNumber: function ordinalNumber(dirtyNumber, _options) {
              var number = Number(dirtyNumber),
                b = number >= 100 ? 100 : null;
              return (
                number + (suffixes[number] || suffixes[number % 10] || (b && suffixes[b]) || '')
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['б.з.д.', 'б.з.'],
                abbreviated: ['б.з.д.', 'б.з.'],
                wide: ['біздің заманымызға дейін', 'біздің заманымыз'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-ші тоқ.', '2-ші тоқ.', '3-ші тоқ.', '4-ші тоқ.'],
                wide: ['1-ші тоқсан', '2-ші тоқсан', '3-ші тоқсан', '4-ші тоқсан'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['Қ', 'А', 'Н', 'С', 'М', 'М', 'Ш', 'Т', 'Қ', 'Қ', 'Қ', 'Ж'],
                abbreviated: [
                  'қаң',
                  'ақп',
                  'нау',
                  'сәу',
                  'мам',
                  'мау',
                  'шіл',
                  'там',
                  'қыр',
                  'қаз',
                  'қар',
                  'жел',
                ],
                wide: [
                  'қаңтар',
                  'ақпан',
                  'наурыз',
                  'сәуір',
                  'мамыр',
                  'маусым',
                  'шілде',
                  'тамыз',
                  'қыркүйек',
                  'қазан',
                  'қараша',
                  'желтоқсан',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['Қ', 'А', 'Н', 'С', 'М', 'М', 'Ш', 'Т', 'Қ', 'Қ', 'Қ', 'Ж'],
                abbreviated: [
                  'қаң',
                  'ақп',
                  'нау',
                  'сәу',
                  'мам',
                  'мау',
                  'шіл',
                  'там',
                  'қыр',
                  'қаз',
                  'қар',
                  'жел',
                ],
                wide: [
                  'қаңтар',
                  'ақпан',
                  'наурыз',
                  'сәуір',
                  'мамыр',
                  'маусым',
                  'шілде',
                  'тамыз',
                  'қыркүйек',
                  'қазан',
                  'қараша',
                  'желтоқсан',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Ж', 'Д', 'С', 'С', 'Б', 'Ж', 'С'],
                short: ['жс', 'дс', 'сс', 'ср', 'бс', 'жм', 'сб'],
                abbreviated: ['жс', 'дс', 'сс', 'ср', 'бс', 'жм', 'сб'],
                wide: ['жексенбі', 'дүйсенбі', 'сейсенбі', 'сәрсенбі', 'бейсенбі', 'жұма', 'сенбі'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасы',
                  noon: 'түс',
                  morning: 'таң',
                  afternoon: 'күндіз',
                  evening: 'кеш',
                  night: 'түн',
                },
                wide: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасы',
                  noon: 'түс',
                  morning: 'таң',
                  afternoon: 'күндіз',
                  evening: 'кеш',
                  night: 'түн',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                narrow: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасында',
                  noon: 'түс',
                  morning: 'таң',
                  afternoon: 'күн',
                  evening: 'кеш',
                  night: 'түн',
                },
                wide: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасында',
                  noon: 'түсте',
                  morning: 'таңертең',
                  afternoon: 'күндіз',
                  evening: 'кеште',
                  night: 'түнде',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/match/index.js':
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
              matchPattern: /^(\d+)(-?(ші|шы))?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^((б )?з\.?\s?д\.?)/i,
                abbreviated: /^((б )?з\.?\s?д\.?)/i,
                wide: /^(біздің заманымызға дейін|біздің заманымыз|біздің заманымыздан)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^б/i, /^з/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234](-?ші)? тоқ.?/i,
                wide: /^[1234](-?ші)? тоқсан/i,
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
                narrow: /^(қ|а|н|с|м|мау|ш|т|қыр|қаз|қар|ж)/i,
                abbreviated: /^(қаң|ақп|нау|сәу|мам|мау|шіл|там|қыр|қаз|қар|жел)/i,
                wide: /^(қаңтар|ақпан|наурыз|сәуір|мамыр|маусым|шілде|тамыз|қыркүйек|қазан|қараша|желтоқсан)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^қ/i,
                  /^а/i,
                  /^н/i,
                  /^с/i,
                  /^м/i,
                  /^м/i,
                  /^ш/i,
                  /^т/i,
                  /^қ/i,
                  /^қ/i,
                  /^қ/i,
                  /^ж/i,
                ],
                abbreviated: [
                  /^қаң/i,
                  /^ақп/i,
                  /^нау/i,
                  /^сәу/i,
                  /^мам/i,
                  /^мау/i,
                  /^шіл/i,
                  /^там/i,
                  /^қыр/i,
                  /^қаз/i,
                  /^қар/i,
                  /^жел/i,
                ],
                any: [
                  /^қ/i,
                  /^а/i,
                  /^н/i,
                  /^с/i,
                  /^м/i,
                  /^м/i,
                  /^ш/i,
                  /^т/i,
                  /^қ/i,
                  /^қ/i,
                  /^қ/i,
                  /^ж/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ж|д|с|с|б|ж|с)/i,
                short: /^(жс|дс|сс|ср|бс|жм|сб)/i,
                wide: /^(жексенбі|дүйсенбі|сейсенбі|сәрсенбі|бейсенбі|жұма|сенбі)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^ж/i, /^д/i, /^с/i, /^с/i, /^б/i, /^ж/i, /^с/i],
                short: [/^жс/i, /^дс/i, /^сс/i, /^ср/i, /^бс/i, /^жм/i, /^сб/i],
                any: [/^ж[ек]/i, /^д[үй]/i, /^сe[й]/i, /^сә[р]/i, /^б[ей]/i, /^ж[ұм]/i, /^се[н]/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow:
                  /^Т\.?\s?[ДК]\.?|түн ортасында|((түсте|таңертең|таңда|таңертең|таңмен|таң|күндіз|күн|кеште|кеш|түнде|түн)\.?)/i,
                wide: /^Т\.?\s?[ДК]\.?|түн ортасында|((түсте|таңертең|таңда|таңертең|таңмен|таң|күндіз|күн|кеште|кеш|түнде|түн)\.?)/i,
                any: /^Т\.?\s?[ДК]\.?|түн ортасында|((түсте|таңертең|таңда|таңертең|таңмен|таң|күндіз|күн|кеште|кеш|түнде|түн)\.?)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: {
                  am: /^ТД/i,
                  pm: /^ТК/i,
                  midnight: /^түн орта/i,
                  noon: /^күндіз/i,
                  morning: /таң/i,
                  afternoon: /түс/i,
                  evening: /кеш/i,
                  night: /түн/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'kk',
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
