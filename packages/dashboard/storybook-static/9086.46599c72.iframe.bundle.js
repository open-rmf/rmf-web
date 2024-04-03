(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [9086, 45585, 69107, 95565, 79489, 31188, 55013, 15751, 57799, 12041],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'mindre än en sekund', other: 'mindre än {{count}} sekunder' },
            xSeconds: { one: 'en sekund', other: '{{count}} sekunder' },
            halfAMinute: 'en halv minut',
            lessThanXMinutes: { one: 'mindre än en minut', other: 'mindre än {{count}} minuter' },
            xMinutes: { one: 'en minut', other: '{{count}} minuter' },
            aboutXHours: { one: 'ungefär en timme', other: 'ungefär {{count}} timmar' },
            xHours: { one: 'en timme', other: '{{count}} timmar' },
            xDays: { one: 'en dag', other: '{{count}} dagar' },
            aboutXWeeks: { one: 'ungefär en vecka', other: 'ungefär {{count}} vecka' },
            xWeeks: { one: 'en vecka', other: '{{count}} vecka' },
            aboutXMonths: { one: 'ungefär en månad', other: 'ungefär {{count}} månader' },
            xMonths: { one: 'en månad', other: '{{count}} månader' },
            aboutXYears: { one: 'ungefär ett år', other: 'ungefär {{count}} år' },
            xYears: { one: 'ett år', other: '{{count}} år' },
            overXYears: { one: 'över ett år', other: 'över {{count}} år' },
            almostXYears: { one: 'nästan ett år', other: 'nästan {{count}} år' },
          },
          wordMapping = [
            'noll',
            'en',
            'två',
            'tre',
            'fyra',
            'fem',
            'sex',
            'sju',
            'åtta',
            'nio',
            'tio',
            'elva',
            'tolv',
          ],
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : options && options.onlyNumeric
                      ? tokenValue.other.replace('{{count}}', String(count))
                      : tokenValue.other.replace(
                          '{{count}}',
                          count < 13 ? wordMapping[count] : String(count),
                        )),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'om ' + result
                  : result + ' sedan'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatLong/index.js':
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
                full: 'EEEE d MMMM y',
                long: 'd MMMM y',
                medium: 'd MMM y',
                short: 'y-MM-dd',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: "'kl'. HH:mm:ss zzzz",
                long: 'HH:mm:ss z',
                medium: 'HH:mm:ss',
                short: 'HH:mm',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: "{{date}} 'kl.' {{time}}",
                long: "{{date}} 'kl.' {{time}}",
                medium: '{{date}} {{time}}',
                short: '{{date}} {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'i' EEEE's kl.' p",
            yesterday: "'igår kl.' p",
            today: "'idag kl.' p",
            tomorrow: "'imorgon kl.' p",
            nextWeek: "EEEE 'kl.' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/localize/index.js':
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
                  case 2:
                    return number + ':a';
                }
              return number + ':e';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['f.Kr.', 'e.Kr.'],
                abbreviated: ['f.Kr.', 'e.Kr.'],
                wide: ['före Kristus', 'efter Kristus'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1:a kvartalet', '2:a kvartalet', '3:e kvartalet', '4:e kvartalet'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['J', 'F', 'M', 'A', 'M', 'J', 'J', 'A', 'S', 'O', 'N', 'D'],
                abbreviated: [
                  'jan.',
                  'feb.',
                  'mars',
                  'apr.',
                  'maj',
                  'juni',
                  'juli',
                  'aug.',
                  'sep.',
                  'okt.',
                  'nov.',
                  'dec.',
                ],
                wide: [
                  'januari',
                  'februari',
                  'mars',
                  'april',
                  'maj',
                  'juni',
                  'juli',
                  'augusti',
                  'september',
                  'oktober',
                  'november',
                  'december',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['S', 'M', 'T', 'O', 'T', 'F', 'L'],
                short: ['sö', 'må', 'ti', 'on', 'to', 'fr', 'lö'],
                abbreviated: ['sön', 'mån', 'tis', 'ons', 'tors', 'fre', 'lör'],
                wide: ['söndag', 'måndag', 'tisdag', 'onsdag', 'torsdag', 'fredag', 'lördag'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'fm',
                  pm: 'em',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'morg.',
                  afternoon: 'efterm.',
                  evening: 'kväll',
                  night: 'natt',
                },
                abbreviated: {
                  am: 'f.m.',
                  pm: 'e.m.',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'morgon',
                  afternoon: 'efterm.',
                  evening: 'kväll',
                  night: 'natt',
                },
                wide: {
                  am: 'förmiddag',
                  pm: 'eftermiddag',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'morgon',
                  afternoon: 'eftermiddag',
                  evening: 'kväll',
                  night: 'natt',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'fm',
                  pm: 'em',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'på morg.',
                  afternoon: 'på efterm.',
                  evening: 'på kvällen',
                  night: 'på natten',
                },
                abbreviated: {
                  am: 'fm',
                  pm: 'em',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'på morg.',
                  afternoon: 'på efterm.',
                  evening: 'på kvällen',
                  night: 'på natten',
                },
                wide: {
                  am: 'fm',
                  pm: 'em',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'på morgonen',
                  afternoon: 'på eftermiddagen',
                  evening: 'på kvällen',
                  night: 'på natten',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/match/index.js':
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
              matchPattern: /^(\d+)(:a|:e)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(f\.? ?Kr\.?|f\.? ?v\.? ?t\.?|e\.? ?Kr\.?|v\.? ?t\.?)/i,
                abbreviated: /^(f\.? ?Kr\.?|f\.? ?v\.? ?t\.?|e\.? ?Kr\.?|v\.? ?t\.?)/i,
                wide: /^(före Kristus|före vår tid|efter Kristus|vår tid)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^f/i, /^[ev]/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^q[1234]/i,
                wide: /^[1234](:a|:e)? kvartalet/i,
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
                abbreviated: /^(jan|feb|mar[s]?|apr|maj|jun[i]?|jul[i]?|aug|sep|okt|nov|dec)\.?/i,
                wide: /^(januari|februari|mars|april|maj|juni|juli|augusti|september|oktober|november|december)/i,
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
                  /^maj/i,
                  /^jun/i,
                  /^jul/i,
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
                narrow: /^[smtofl]/i,
                short: /^(sö|må|ti|on|to|fr|lö)/i,
                abbreviated: /^(sön|mån|tis|ons|tors|fre|lör)/i,
                wide: /^(söndag|måndag|tisdag|onsdag|torsdag|fredag|lördag)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^s/i, /^m/i, /^ti/i, /^o/i, /^to/i, /^f/i, /^l/i] },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                any: /^([fe]\.?\s?m\.?|midn(att)?|midd(ag)?|(på) (morgonen|eftermiddagen|kvällen|natten))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^f/i,
                  pm: /^e/i,
                  midnight: /^midn/i,
                  noon: /^midd/i,
                  morning: /morgon/i,
                  afternoon: /eftermiddag/i,
                  evening: /kväll/i,
                  night: /natt/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sv/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'sv',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 1, firstWeekContainsDate: 4 },
        };
      (exports.default = _default), (module.exports = exports.default);
    },
  },
]);
