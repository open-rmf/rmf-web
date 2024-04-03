(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [52281, 45585, 69107, 95565, 79489, 1491, 38750, 39628, 69364, 11156],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: '1 сониядан кам', other: '{{count}} сониядан кам' },
            xSeconds: { one: '1 сония', other: '{{count}} сония' },
            halfAMinute: 'ярим дақиқа',
            lessThanXMinutes: { one: '1 дақиқадан кам', other: '{{count}} дақиқадан кам' },
            xMinutes: { one: '1 дақиқа', other: '{{count}} дақиқа' },
            aboutXHours: { one: 'тахминан 1 соат', other: 'тахминан {{count}} соат' },
            xHours: { one: '1 соат', other: '{{count}} соат' },
            xDays: { one: '1 кун', other: '{{count}} кун' },
            aboutXWeeks: { one: 'тахминан 1 хафта', other: 'тахминан {{count}} хафта' },
            xWeeks: { one: '1 хафта', other: '{{count}} хафта' },
            aboutXMonths: { one: 'тахминан 1 ой', other: 'тахминан {{count}} ой' },
            xMonths: { one: '1 ой', other: '{{count}} ой' },
            aboutXYears: { one: 'тахминан 1 йил', other: 'тахминан {{count}} йил' },
            xYears: { one: '1 йил', other: '{{count}} йил' },
            overXYears: { one: '1 йилдан кўп', other: '{{count}} йилдан кўп' },
            almostXYears: { one: 'деярли 1 йил', other: 'деярли {{count}} йил' },
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
                  ? result + 'дан кейин'
                  : result + ' олдин'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatLong/index.js':
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
                full: 'EEEE, do MMMM, y',
                long: 'do MMMM, y',
                medium: 'd MMM, y',
                short: 'dd/MM/yyyy',
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'ўтган' eeee p 'да'",
            yesterday: "'кеча' p 'да'",
            today: "'бугун' p 'да'",
            tomorrow: "'эртага' p 'да'",
            nextWeek: "eeee p 'да'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/localize/index.js':
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
                narrow: ['М.А', 'М'],
                abbreviated: ['М.А', 'М'],
                wide: ['Милоддан Аввалги', 'Милодий'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-чор.', '2-чор.', '3-чор.', '4-чор.'],
                wide: ['1-чорак', '2-чорак', '3-чорак', '4-чорак'],
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
                  'янв',
                  'фев',
                  'мар',
                  'апр',
                  'май',
                  'июн',
                  'июл',
                  'авг',
                  'сен',
                  'окт',
                  'ноя',
                  'дек',
                ],
                wide: [
                  'январ',
                  'феврал',
                  'март',
                  'апрел',
                  'май',
                  'июн',
                  'июл',
                  'август',
                  'сентабр',
                  'октабр',
                  'ноябр',
                  'декабр',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Я', 'Д', 'С', 'Ч', 'П', 'Ж', 'Ш'],
                short: ['як', 'ду', 'се', 'чо', 'па', 'жу', 'ша'],
                abbreviated: ['якш', 'душ', 'сеш', 'чор', 'пай', 'жум', 'шан'],
                wide: ['якшанба', 'душанба', 'сешанба', 'чоршанба', 'пайшанба', 'жума', 'шанба'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                any: {
                  am: 'П.О.',
                  pm: 'П.К.',
                  midnight: 'ярим тун',
                  noon: 'пешин',
                  morning: 'эрталаб',
                  afternoon: 'пешиндан кейин',
                  evening: 'кечаси',
                  night: 'тун',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                any: {
                  am: 'П.О.',
                  pm: 'П.К.',
                  midnight: 'ярим тун',
                  noon: 'пешин',
                  morning: 'эрталаб',
                  afternoon: 'пешиндан кейин',
                  evening: 'кечаси',
                  night: 'тун',
                },
              },
              defaultFormattingWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/match/index.js':
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
              matchPattern: /^(\d+)(чи)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(м\.а|м\.)/i,
                abbreviated: /^(м\.а|м\.)/i,
                wide: /^(милоддан аввал|милоддан кейин)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^м/i, /^а/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234]-чор./i,
                wide: /^[1234]-чорак/i,
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
                narrow: /^[яфмамииасонд]/i,
                abbreviated: /^(янв|фев|мар|апр|май|июн|июл|авг|сен|окт|ноя|дек)/i,
                wide: /^(январ|феврал|март|апрел|май|июн|июл|август|сентабр|октабр|ноябр|декабр)/i,
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
                  /^д/i,
                ],
                any: [
                  /^я/i,
                  /^ф/i,
                  /^мар/i,
                  /^ап/i,
                  /^май/i,
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
                narrow: /^[ядсчпжш]/i,
                short: /^(як|ду|се|чо|па|жу|ша)/i,
                abbreviated: /^(якш|душ|сеш|чор|пай|жум|шан)/i,
                wide: /^(якшанба|душанба|сешанба|чоршанба|пайшанба|жума|шанба)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^я/i, /^д/i, /^с/i, /^ч/i, /^п/i, /^ж/i, /^ш/i],
                any: [/^як/i, /^ду/i, /^се/i, /^чор/i, /^пай/i, /^жу/i, /^шан/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                any: /^(п\.о\.|п\.к\.|ярим тун|пешиндан кейин|(эрталаб|пешиндан кейин|кечаси|тун))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^п\.о\./i,
                  pm: /^п\.к\./i,
                  midnight: /^ярим тун/i,
                  noon: /^пешиндан кейин/i,
                  morning: /эрталаб/i,
                  afternoon: /пешиндан кейин/i,
                  evening: /кечаси/i,
                  night: /тун/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'uz-Cyrl',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 1, firstWeekContainsDate: 1 },
        };
      (exports.default = _default), (module.exports = exports.default);
    },
  },
]);
