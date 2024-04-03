(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [43983, 45585, 69107, 95565, 79489, 50721, 83084, 80582, 5558, 92658],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'llai na eiliad', other: 'llai na {{count}} eiliad' },
            xSeconds: { one: '1 eiliad', other: '{{count}} eiliad' },
            halfAMinute: 'hanner munud',
            lessThanXMinutes: {
              one: 'llai na munud',
              two: 'llai na 2 funud',
              other: 'llai na {{count}} munud',
            },
            xMinutes: { one: '1 munud', two: '2 funud', other: '{{count}} munud' },
            aboutXHours: { one: 'tua 1 awr', other: 'tua {{count}} awr' },
            xHours: { one: '1 awr', other: '{{count}} awr' },
            xDays: { one: '1 diwrnod', two: '2 ddiwrnod', other: '{{count}} diwrnod' },
            aboutXWeeks: {
              one: 'tua 1 wythnos',
              two: 'tua pythefnos',
              other: 'tua {{count}} wythnos',
            },
            xWeeks: { one: '1 wythnos', two: 'pythefnos', other: '{{count}} wythnos' },
            aboutXMonths: { one: 'tua 1 mis', two: 'tua 2 fis', other: 'tua {{count}} mis' },
            xMonths: { one: '1 mis', two: '2 fis', other: '{{count}} mis' },
            aboutXYears: {
              one: 'tua 1 flwyddyn',
              two: 'tua 2 flynedd',
              other: 'tua {{count}} mlynedd',
            },
            xYears: { one: '1 flwyddyn', two: '2 flynedd', other: '{{count}} mlynedd' },
            overXYears: {
              one: 'dros 1 flwyddyn',
              two: 'dros 2 flynedd',
              other: 'dros {{count}} mlynedd',
            },
            almostXYears: {
              one: 'bron 1 flwyddyn',
              two: 'bron 2 flynedd',
              other: 'bron {{count}} mlynedd',
            },
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
                    : 2 === count && tokenValue.two
                      ? tokenValue.two
                      : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'mewn ' + result
                  : result + ' yn ôl'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatLong/index.js':
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
                full: 'EEEE, d MMMM yyyy',
                long: 'd MMMM yyyy',
                medium: 'd MMM yyyy',
                short: 'dd/MM/yyyy',
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
                full: "{{date}} 'am' {{time}}",
                long: "{{date}} 'am' {{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'diwethaf am' p",
            yesterday: "'ddoe am' p",
            today: "'heddiw am' p",
            tomorrow: "'yfory am' p",
            nextWeek: "eeee 'am' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/localize/index.js':
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
              var number = Number(dirtyNumber);
              if (number < 20)
                switch (number) {
                  case 0:
                  case 7:
                  case 8:
                  case 9:
                  case 10:
                  case 12:
                  case 15:
                  case 18:
                    return number + 'fed';
                  case 1:
                    return number + 'af';
                  case 2:
                    return number + 'ail';
                  case 3:
                  case 4:
                    return number + 'ydd';
                  case 5:
                  case 6:
                    return number + 'ed';
                  case 11:
                  case 13:
                  case 14:
                  case 16:
                  case 17:
                  case 19:
                    return number + 'eg';
                }
              else if ((number >= 50 && number <= 60) || 80 === number || number >= 100)
                return number + 'fed';
              return number + 'ain';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['C', 'O'],
                abbreviated: ['CC', 'OC'],
                wide: ['Cyn Crist', 'Ar ôl Crist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Ch1', 'Ch2', 'Ch3', 'Ch4'],
                wide: ['Chwarter 1af', '2ail chwarter', '3ydd chwarter', '4ydd chwarter'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['I', 'Ch', 'Ma', 'E', 'Mi', 'Me', 'G', 'A', 'Md', 'H', 'T', 'Rh'],
                abbreviated: [
                  'Ion',
                  'Chwe',
                  'Maw',
                  'Ebr',
                  'Mai',
                  'Meh',
                  'Gor',
                  'Aws',
                  'Med',
                  'Hyd',
                  'Tach',
                  'Rhag',
                ],
                wide: [
                  'Ionawr',
                  'Chwefror',
                  'Mawrth',
                  'Ebrill',
                  'Mai',
                  'Mehefin',
                  'Gorffennaf',
                  'Awst',
                  'Medi',
                  'Hydref',
                  'Tachwedd',
                  'Rhagfyr',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['S', 'Ll', 'M', 'M', 'I', 'G', 'S'],
                short: ['Su', 'Ll', 'Ma', 'Me', 'Ia', 'Gw', 'Sa'],
                abbreviated: ['Sul', 'Llun', 'Maw', 'Mer', 'Iau', 'Gwe', 'Sad'],
                wide: [
                  'dydd Sul',
                  'dydd Llun',
                  'dydd Mawrth',
                  'dydd Mercher',
                  'dydd Iau',
                  'dydd Gwener',
                  'dydd Sadwrn',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'b',
                  pm: 'h',
                  midnight: 'hn',
                  noon: 'hd',
                  morning: 'bore',
                  afternoon: 'prynhawn',
                  evening: "gyda'r nos",
                  night: 'nos',
                },
                abbreviated: {
                  am: 'yb',
                  pm: 'yh',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'bore',
                  afternoon: 'prynhawn',
                  evening: "gyda'r nos",
                  night: 'nos',
                },
                wide: {
                  am: 'y.b.',
                  pm: 'y.h.',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'bore',
                  afternoon: 'prynhawn',
                  evening: "gyda'r nos",
                  night: 'nos',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'b',
                  pm: 'h',
                  midnight: 'hn',
                  noon: 'hd',
                  morning: 'yn y bore',
                  afternoon: 'yn y prynhawn',
                  evening: "gyda'r nos",
                  night: 'yn y nos',
                },
                abbreviated: {
                  am: 'yb',
                  pm: 'yh',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'yn y bore',
                  afternoon: 'yn y prynhawn',
                  evening: "gyda'r nos",
                  night: 'yn y nos',
                },
                wide: {
                  am: 'y.b.',
                  pm: 'y.h.',
                  midnight: 'hanner nos',
                  noon: 'hanner dydd',
                  morning: 'yn y bore',
                  afternoon: 'yn y prynhawn',
                  evening: "gyda'r nos",
                  night: 'yn y nos',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/match/index.js':
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
              matchPattern: /^(\d+)(af|ail|ydd|ed|fed|eg|ain)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(c|o)/i,
                abbreviated: /^(c\.?\s?c\.?|o\.?\s?c\.?)/i,
                wide: /^(cyn christ|ar ôl crist|ar ol crist)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { wide: [/^c/i, /^(ar ôl crist|ar ol crist)/i], any: [/^c/i, /^o/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^ch[1234]/i,
                wide: /^(chwarter 1af)|([234](ail|ydd)? chwarter)/i,
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
                narrow: /^(i|ch|m|e|g|a|h|t|rh)/i,
                abbreviated: /^(ion|chwe|maw|ebr|mai|meh|gor|aws|med|hyd|tach|rhag)/i,
                wide: /^(ionawr|chwefror|mawrth|ebrill|mai|mehefin|gorffennaf|awst|medi|hydref|tachwedd|rhagfyr)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^i/i,
                  /^ch/i,
                  /^m/i,
                  /^e/i,
                  /^m/i,
                  /^m/i,
                  /^g/i,
                  /^a/i,
                  /^m/i,
                  /^h/i,
                  /^t/i,
                  /^rh/i,
                ],
                any: [
                  /^io/i,
                  /^ch/i,
                  /^maw/i,
                  /^e/i,
                  /^mai/i,
                  /^meh/i,
                  /^g/i,
                  /^a/i,
                  /^med/i,
                  /^h/i,
                  /^t/i,
                  /^rh/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(s|ll|m|i|g)/i,
                short: /^(su|ll|ma|me|ia|gw|sa)/i,
                abbreviated: /^(sul|llun|maw|mer|iau|gwe|sad)/i,
                wide: /^dydd (sul|llun|mawrth|mercher|iau|gwener|sadwrn)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^s/i, /^ll/i, /^m/i, /^m/i, /^i/i, /^g/i, /^s/i],
                wide: [
                  /^dydd su/i,
                  /^dydd ll/i,
                  /^dydd ma/i,
                  /^dydd me/i,
                  /^dydd i/i,
                  /^dydd g/i,
                  /^dydd sa/i,
                ],
                any: [/^su/i, /^ll/i, /^ma/i, /^me/i, /^i/i, /^g/i, /^sa/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(b|h|hn|hd|(yn y|y|yr|gyda'r) (bore|prynhawn|nos|hwyr))/i,
                any: /^(y\.?\s?[bh]\.?|hanner nos|hanner dydd|(yn y|y|yr|gyda'r) (bore|prynhawn|nos|hwyr))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^b|(y\.?\s?b\.?)/i,
                  pm: /^h|(y\.?\s?h\.?)|(yr hwyr)/i,
                  midnight: /^hn|hanner nos/i,
                  noon: /^hd|hanner dydd/i,
                  morning: /bore/i,
                  afternoon: /prynhawn/i,
                  evening: /^gyda'r nos$/i,
                  night: /blah/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'cy',
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
