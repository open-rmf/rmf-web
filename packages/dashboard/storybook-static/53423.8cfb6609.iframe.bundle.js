(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [53423, 45585, 69107, 95565, 79489, 79425, 75756, 94310, 70870, 26450],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'помалку од секунда', other: 'помалку од {{count}} секунди' },
            xSeconds: { one: '1 секунда', other: '{{count}} секунди' },
            halfAMinute: 'половина минута',
            lessThanXMinutes: { one: 'помалку од минута', other: 'помалку од {{count}} минути' },
            xMinutes: { one: '1 минута', other: '{{count}} минути' },
            aboutXHours: { one: 'околу 1 час', other: 'околу {{count}} часа' },
            xHours: { one: '1 час', other: '{{count}} часа' },
            xDays: { one: '1 ден', other: '{{count}} дена' },
            aboutXWeeks: { one: 'околу 1 недела', other: 'околу {{count}} месеци' },
            xWeeks: { one: '1 недела', other: '{{count}} недели' },
            aboutXMonths: { one: 'околу 1 месец', other: 'околу {{count}} недели' },
            xMonths: { one: '1 месец', other: '{{count}} месеци' },
            aboutXYears: { one: 'околу 1 година', other: 'околу {{count}} години' },
            xYears: { one: '1 година', other: '{{count}} години' },
            overXYears: { one: 'повеќе од 1 година', other: 'повеќе од {{count}} години' },
            almostXYears: { one: 'безмалку 1 година', other: 'безмалку {{count}} години' },
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
                  ? 'за ' + result
                  : 'пред ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatLong/index.js':
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatRelative/index.js':
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
          weekdays = ['недела', 'понеделник', 'вторник', 'среда', 'четврток', 'петок', 'сабота'];
        function thisWeek(day) {
          var weekday = weekdays[day];
          switch (day) {
            case 0:
            case 3:
            case 6:
              return "'ова " + weekday + " вo' p";
            case 1:
            case 2:
            case 4:
            case 5:
              return "'овој " + weekday + " вo' p";
          }
        }
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date, baseDate, options) {
              var day = date.getUTCDay();
              return (0, _index.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function _lastWeek(day) {
                    var weekday = weekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 6:
                        return "'минатата " + weekday + " во' p";
                      case 1:
                      case 2:
                      case 4:
                      case 5:
                        return "'минатиот " + weekday + " во' p";
                    }
                  })(day);
            },
            yesterday: "'вчера во' p",
            today: "'денес во' p",
            tomorrow: "'утре во' p",
            nextWeek: function nextWeek(date, baseDate, options) {
              var day = date.getUTCDay();
              return (0, _index.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function _nextWeek(day) {
                    var weekday = weekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 6:
                        return "'следната " + weekday + " вo' p";
                      case 1:
                      case 2:
                      case 4:
                      case 5:
                        return "'следниот " + weekday + " вo' p";
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/localize/index.js':
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
              var number = Number(dirtyNumber),
                rem100 = number % 100;
              if (rem100 > 20 || rem100 < 10)
                switch (rem100 % 10) {
                  case 1:
                    return number + '-ви';
                  case 2:
                    return number + '-ри';
                  case 7:
                  case 8:
                    return number + '-ми';
                }
              return number + '-ти';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['пр.н.е.', 'н.е.'],
                abbreviated: ['пред н. е.', 'н. е.'],
                wide: ['пред нашата ера', 'нашата ера'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-ви кв.', '2-ри кв.', '3-ти кв.', '4-ти кв.'],
                wide: ['1-ви квартал', '2-ри квартал', '3-ти квартал', '4-ти квартал'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                abbreviated: [
                  'јан',
                  'фев',
                  'мар',
                  'апр',
                  'мај',
                  'јун',
                  'јул',
                  'авг',
                  'септ',
                  'окт',
                  'ноем',
                  'дек',
                ],
                wide: [
                  'јануари',
                  'февруари',
                  'март',
                  'април',
                  'мај',
                  'јуни',
                  'јули',
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
                short: ['не', 'по', 'вт', 'ср', 'че', 'пе', 'са'],
                abbreviated: ['нед', 'пон', 'вто', 'сре', 'чет', 'пет', 'саб'],
                wide: ['недела', 'понеделник', 'вторник', 'среда', 'четврток', 'петок', 'сабота'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                wide: {
                  am: 'претпладне',
                  pm: 'попладне',
                  midnight: 'полноќ',
                  noon: 'напладне',
                  morning: 'наутро',
                  afternoon: 'попладне',
                  evening: 'навечер',
                  night: 'ноќе',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/match/index.js':
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
              matchPattern: /^(\d+)(-?[врмт][и])?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^((пр)?н\.?\s?е\.?)/i,
                abbreviated: /^((пр)?н\.?\s?е\.?)/i,
                wide: /^(пред нашата ера|нашата ера)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^п/i, /^н/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234](-?[врт]?и?)? кв.?/i,
                wide: /^[1234](-?[врт]?и?)? квартал/i,
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
                abbreviated: /^(јан|фев|мар|апр|мај|јун|јул|авг|сеп|окт|ноем|дек)/i,
                wide: /^(јануари|февруари|март|април|мај|јуни|јули|август|септември|октомври|ноември|декември)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: [
                  /^ја/i,
                  /^Ф/i,
                  /^мар/i,
                  /^ап/i,
                  /^мај/i,
                  /^јун/i,
                  /^јул/i,
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
                short: /^(не|по|вт|ср|че|пе|са)/i,
                abbreviated: /^(нед|пон|вто|сре|чет|пет|саб)/i,
                wide: /^(недела|понеделник|вторник|среда|четврток|петок|сабота)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^н/i, /^п/i, /^в/i, /^с/i, /^ч/i, /^п/i, /^с/i],
                any: [/^н[ед]/i, /^п[он]/i, /^вт/i, /^ср/i, /^ч[ет]/i, /^п[ет]/i, /^с[аб]/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: { any: /^(претп|попл|полноќ|утро|пладне|вечер|ноќ)/i },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /претпладне/i,
                  pm: /попладне/i,
                  midnight: /полноќ/i,
                  noon: /напладне/i,
                  morning: /наутро/i,
                  afternoon: /попладне/i,
                  evening: /навечер/i,
                  night: /ноќе/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'mk',
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
