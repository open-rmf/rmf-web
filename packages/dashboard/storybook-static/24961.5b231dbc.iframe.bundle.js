(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [24961, 45585, 69107, 95565, 79489, 13355, 26950, 88484, 37756, 94492],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        function declension(scheme, count, time) {
          var group = (function declensionGroup(scheme, count) {
            return 1 === count && scheme.one
              ? scheme.one
              : count >= 2 && count <= 4 && scheme.twoFour
                ? scheme.twoFour
                : scheme.other;
          })(scheme, count);
          return group[time].replace('{{count}}', String(count));
        }
        function prefixPreposition(preposition) {
          var translation = '';
          return (
            'almost' === preposition && (translation = 'takmer'),
            'about' === preposition && (translation = 'približne'),
            translation.length > 0 ? translation + ' ' : ''
          );
        }
        function suffixPreposition(preposition) {
          var translation = '';
          return (
            'lessThan' === preposition && (translation = 'menej než'),
            'over' === preposition && (translation = 'viac než'),
            translation.length > 0 ? translation + ' ' : ''
          );
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            xSeconds: {
              one: { present: 'sekunda', past: 'sekundou', future: 'sekundu' },
              twoFour: {
                present: '{{count}} sekundy',
                past: '{{count}} sekundami',
                future: '{{count}} sekundy',
              },
              other: {
                present: '{{count}} sekúnd',
                past: '{{count}} sekundami',
                future: '{{count}} sekúnd',
              },
            },
            halfAMinute: {
              other: { present: 'pol minúty', past: 'pol minútou', future: 'pol minúty' },
            },
            xMinutes: {
              one: { present: 'minúta', past: 'minútou', future: 'minútu' },
              twoFour: {
                present: '{{count}} minúty',
                past: '{{count}} minútami',
                future: '{{count}} minúty',
              },
              other: {
                present: '{{count}} minút',
                past: '{{count}} minútami',
                future: '{{count}} minút',
              },
            },
            xHours: {
              one: { present: 'hodina', past: 'hodinou', future: 'hodinu' },
              twoFour: {
                present: '{{count}} hodiny',
                past: '{{count}} hodinami',
                future: '{{count}} hodiny',
              },
              other: {
                present: '{{count}} hodín',
                past: '{{count}} hodinami',
                future: '{{count}} hodín',
              },
            },
            xDays: {
              one: { present: 'deň', past: 'dňom', future: 'deň' },
              twoFour: {
                present: '{{count}} dni',
                past: '{{count}} dňami',
                future: '{{count}} dni',
              },
              other: { present: '{{count}} dní', past: '{{count}} dňami', future: '{{count}} dní' },
            },
            xWeeks: {
              one: { present: 'týždeň', past: 'týždňom', future: 'týždeň' },
              twoFour: {
                present: '{{count}} týždne',
                past: '{{count}} týždňami',
                future: '{{count}} týždne',
              },
              other: {
                present: '{{count}} týždňov',
                past: '{{count}} týždňami',
                future: '{{count}} týždňov',
              },
            },
            xMonths: {
              one: { present: 'mesiac', past: 'mesiacom', future: 'mesiac' },
              twoFour: {
                present: '{{count}} mesiace',
                past: '{{count}} mesiacmi',
                future: '{{count}} mesiace',
              },
              other: {
                present: '{{count}} mesiacov',
                past: '{{count}} mesiacmi',
                future: '{{count}} mesiacov',
              },
            },
            xYears: {
              one: { present: 'rok', past: 'rokom', future: 'rok' },
              twoFour: {
                present: '{{count}} roky',
                past: '{{count}} rokmi',
                future: '{{count}} roky',
              },
              other: {
                present: '{{count}} rokov',
                past: '{{count}} rokmi',
                future: '{{count}} rokov',
              },
            },
          },
          _default = function formatDistance(token, count, options) {
            var preposition =
                (function extractPreposition(token) {
                  return ['lessThan', 'about', 'over', 'almost'].filter(function (preposition) {
                    return !!token.match(new RegExp('^' + preposition));
                  })[0];
                })(token) || '',
              key = (function lowercaseFirstLetter(string) {
                return string.charAt(0).toLowerCase() + string.slice(1);
              })(token.substring(preposition.length)),
              scheme = formatDistanceLocale[key];
            return null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? prefixPreposition(preposition) +
                  'o ' +
                  suffixPreposition(preposition) +
                  declension(scheme, count, 'future')
                : prefixPreposition(preposition) +
                  'pred ' +
                  suffixPreposition(preposition) +
                  declension(scheme, count, 'past')
              : prefixPreposition(preposition) +
                  suffixPreposition(preposition) +
                  declension(scheme, count, 'present');
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatLong/index.js':
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
                full: 'EEEE d. MMMM y',
                long: 'd. MMMM y',
                medium: 'd. M. y',
                short: 'd. M. y',
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
              formats: {
                full: '{{date}}, {{time}}',
                long: '{{date}}, {{time}}',
                medium: '{{date}}, {{time}}',
                short: '{{date}} {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatRelative/index.js':
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
            'nedeľu',
            'pondelok',
            'utorok',
            'stredu',
            'štvrtok',
            'piatok',
            'sobotu',
          ];
        function thisWeek(day) {
          return 4 === day ? "'vo' eeee 'o' p" : "'v " + accusativeWeekdays[day] + " o' p";
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
                      case 3:
                      case 6:
                        return "'minulú " + weekday + " o' p";
                      default:
                        return "'minulý' eeee 'o' p";
                    }
                  })(day);
            },
            yesterday: "'včera o' p",
            today: "'dnes o' p",
            tomorrow: "'zajtra o' p",
            nextWeek: function nextWeek(date, baseDate, options) {
              var day = date.getUTCDay();
              return (0, _index.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function _nextWeek(day) {
                    var weekday = accusativeWeekdays[day];
                    switch (day) {
                      case 0:
                      case 4:
                      case 6:
                        return "'budúcu " + weekday + " o' p";
                      default:
                        return "'budúci' eeee 'o' p";
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/localize/index.js':
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
              return Number(dirtyNumber) + '.';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['pred Kr.', 'po Kr.'],
                abbreviated: ['pred Kr.', 'po Kr.'],
                wide: ['pred Kristom', 'po Kristovi'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1. štvrťrok', '2. štvrťrok', '3. štvrťrok', '4. štvrťrok'],
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
                  'jan',
                  'feb',
                  'mar',
                  'apr',
                  'máj',
                  'jún',
                  'júl',
                  'aug',
                  'sep',
                  'okt',
                  'nov',
                  'dec',
                ],
                wide: [
                  'január',
                  'február',
                  'marec',
                  'apríl',
                  'máj',
                  'jún',
                  'júl',
                  'august',
                  'september',
                  'október',
                  'november',
                  'december',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['j', 'f', 'm', 'a', 'm', 'j', 'j', 'a', 's', 'o', 'n', 'd'],
                abbreviated: [
                  'jan',
                  'feb',
                  'mar',
                  'apr',
                  'máj',
                  'jún',
                  'júl',
                  'aug',
                  'sep',
                  'okt',
                  'nov',
                  'dec',
                ],
                wide: [
                  'januára',
                  'februára',
                  'marca',
                  'apríla',
                  'mája',
                  'júna',
                  'júla',
                  'augusta',
                  'septembra',
                  'októbra',
                  'novembra',
                  'decembra',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['n', 'p', 'u', 's', 'š', 'p', 's'],
                short: ['ne', 'po', 'ut', 'st', 'št', 'pi', 'so'],
                abbreviated: ['ne', 'po', 'ut', 'st', 'št', 'pi', 'so'],
                wide: ['nedeľa', 'pondelok', 'utorok', 'streda', 'štvrtok', 'piatok', 'sobota'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'poln.',
                  noon: 'pol.',
                  morning: 'ráno',
                  afternoon: 'pop.',
                  evening: 'več.',
                  night: 'noc',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'poln.',
                  noon: 'pol.',
                  morning: 'ráno',
                  afternoon: 'popol.',
                  evening: 'večer',
                  night: 'noc',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'polnoc',
                  noon: 'poludnie',
                  morning: 'ráno',
                  afternoon: 'popoludnie',
                  evening: 'večer',
                  night: 'noc',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'o poln.',
                  noon: 'nap.',
                  morning: 'ráno',
                  afternoon: 'pop.',
                  evening: 'več.',
                  night: 'v n.',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'o poln.',
                  noon: 'napol.',
                  morning: 'ráno',
                  afternoon: 'popol.',
                  evening: 'večer',
                  night: 'v noci',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'o polnoci',
                  noon: 'napoludnie',
                  morning: 'ráno',
                  afternoon: 'popoludní',
                  evening: 'večer',
                  night: 'v noci',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/match/index.js':
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
              matchPattern: /^(\d+)\.?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(pred Kr\.|pred n\. l\.|po Kr\.|n\. l\.)/i,
                abbreviated: /^(pred Kr\.|pred n\. l\.|po Kr\.|n\. l\.)/i,
                wide: /^(pred Kristom|pred na[šs][íi]m letopo[čc]tom|po Kristovi|n[áa][šs]ho letopo[čc]tu)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^pr/i, /^(po|n)/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^q[1234]/i,
                wide: /^[1234]\. [šs]tvr[ťt]rok/i,
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
                narrow: /^[jfmasond]/i,
                abbreviated: /^(jan|feb|mar|apr|m[áa]j|j[úu]n|j[úu]l|aug|sep|okt|nov|dec)/i,
                wide: /^(janu[áa]ra?|febru[áa]ra?|(marec|marca)|apr[íi]la?|m[áa]ja?|j[úu]na?|j[úu]la?|augusta?|(september|septembra)|(okt[óo]ber|okt[óo]bra)|(november|novembra)|(december|decembra))/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^j/i,
                  /^f/i,
                  /^m/i,
                  /^a/i,
                  /^m/i,
                  /^j/i,
                  /^j/i,
                  /^a/i,
                  /^s/i,
                  /^o/i,
                  /^n/i,
                  /^d/i,
                ],
                any: [
                  /^ja/i,
                  /^f/i,
                  /^mar/i,
                  /^ap/i,
                  /^m[áa]j/i,
                  /^j[úu]n/i,
                  /^j[úu]l/i,
                  /^au/i,
                  /^s/i,
                  /^o/i,
                  /^n/i,
                  /^d/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[npusšp]/i,
                short: /^(ne|po|ut|st|št|pi|so)/i,
                abbreviated: /^(ne|po|ut|st|št|pi|so)/i,
                wide: /^(nede[ľl]a|pondelok|utorok|streda|[šs]tvrtok|piatok|sobota])/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^n/i, /^p/i, /^u/i, /^s/i, /^š/i, /^p/i, /^s/i],
                any: [/^n/i, /^po/i, /^u/i, /^st/i, /^(št|stv)/i, /^pi/i, /^so/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow:
                  /^(am|pm|(o )?poln\.?|(nap\.?|pol\.?)|r[áa]no|pop\.?|ve[čc]\.?|(v n\.?|noc))/i,
                abbreviated:
                  /^(am|pm|(o )?poln\.?|(napol\.?|pol\.?)|r[áa]no|pop\.?|ve[čc]er|(v )?noci?)/i,
                any: /^(am|pm|(o )?polnoci?|(na)?poludnie|r[áa]no|popoludn(ie|í|i)|ve[čc]er|(v )?noci?)/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^am/i,
                  pm: /^pm/i,
                  midnight: /poln/i,
                  noon: /^(nap|(na)?pol(\.|u))/i,
                  morning: /^r[áa]no/i,
                  afternoon: /^pop/i,
                  evening: /^ve[čc]/i,
                  night: /^(noc|v n\.)/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'sk',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 1, firstWeekContainsDate: 4 },
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
