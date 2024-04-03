(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [14479, 45585, 69107, 95565, 79489, 76257, 11276, 91142, 63094, 54834],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'menos de un segundo', other: 'menos de {{count}} segundos' },
            xSeconds: { one: '1 segundo', other: '{{count}} segundos' },
            halfAMinute: 'medio minuto',
            lessThanXMinutes: { one: 'menos de un minuto', other: 'menos de {{count}} minutos' },
            xMinutes: { one: '1 minuto', other: '{{count}} minutos' },
            aboutXHours: { one: 'alrededor de 1 hora', other: 'alrededor de {{count}} horas' },
            xHours: { one: '1 hora', other: '{{count}} horas' },
            xDays: { one: '1 día', other: '{{count}} días' },
            aboutXWeeks: { one: 'alrededor de 1 semana', other: 'alrededor de {{count}} semanas' },
            xWeeks: { one: '1 semana', other: '{{count}} semanas' },
            aboutXMonths: { one: 'alrededor de 1 mes', other: 'alrededor de {{count}} meses' },
            xMonths: { one: '1 mes', other: '{{count}} meses' },
            aboutXYears: { one: 'alrededor de 1 año', other: 'alrededor de {{count}} años' },
            xYears: { one: '1 año', other: '{{count}} años' },
            overXYears: { one: 'más de 1 año', other: 'más de {{count}} años' },
            almostXYears: { one: 'casi 1 año', other: 'casi {{count}} años' },
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
                    : tokenValue.other.replace('{{count}}', count.toString())),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'en ' + result
                  : 'hace ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatLong/index.js':
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
                full: "EEEE, d 'de' MMMM 'de' y",
                long: "d 'de' MMMM 'de' y",
                medium: 'd MMM y',
                short: 'dd/MM/y',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'HH:mm:ss zzzz',
                long: 'HH:mm:ss z',
                medium: 'HH:mm:ss',
                short: 'HH:mm',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: "{{date}} 'a las' {{time}}",
                long: "{{date}} 'a las' {{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'el' eeee 'pasado a la' p",
            yesterday: "'ayer a la' p",
            today: "'hoy a la' p",
            tomorrow: "'mañana a la' p",
            nextWeek: "eeee 'a la' p",
            other: 'P',
          },
          formatRelativeLocalePlural = {
            lastWeek: "'el' eeee 'pasado a las' p",
            yesterday: "'ayer a las' p",
            today: "'hoy a las' p",
            tomorrow: "'mañana a las' p",
            nextWeek: "eeee 'a las' p",
            other: 'P',
          },
          _default = function formatRelative(token, date, _baseDate, _options) {
            return 1 !== date.getUTCHours()
              ? formatRelativeLocalePlural[token]
              : formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/localize/index.js':
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
              return Number(dirtyNumber) + 'º';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['AC', 'DC'],
                abbreviated: ['AC', 'DC'],
                wide: ['antes de cristo', 'después de cristo'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['T1', 'T2', 'T3', 'T4'],
                wide: ['1º trimestre', '2º trimestre', '3º trimestre', '4º trimestre'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return Number(quarter) - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['e', 'f', 'm', 'a', 'm', 'j', 'j', 'a', 's', 'o', 'n', 'd'],
                abbreviated: [
                  'ene',
                  'feb',
                  'mar',
                  'abr',
                  'may',
                  'jun',
                  'jul',
                  'ago',
                  'sep',
                  'oct',
                  'nov',
                  'dic',
                ],
                wide: [
                  'enero',
                  'febrero',
                  'marzo',
                  'abril',
                  'mayo',
                  'junio',
                  'julio',
                  'agosto',
                  'septiembre',
                  'octubre',
                  'noviembre',
                  'diciembre',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['d', 'l', 'm', 'm', 'j', 'v', 's'],
                short: ['do', 'lu', 'ma', 'mi', 'ju', 'vi', 'sá'],
                abbreviated: ['dom', 'lun', 'mar', 'mié', 'jue', 'vie', 'sáb'],
                wide: ['domingo', 'lunes', 'martes', 'miércoles', 'jueves', 'viernes', 'sábado'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'mn',
                  noon: 'md',
                  morning: 'mañana',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noche',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'medianoche',
                  noon: 'mediodia',
                  morning: 'mañana',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noche',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'medianoche',
                  noon: 'mediodia',
                  morning: 'mañana',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noche',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'mn',
                  noon: 'md',
                  morning: 'de la mañana',
                  afternoon: 'de la tarde',
                  evening: 'de la tarde',
                  night: 'de la noche',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'medianoche',
                  noon: 'mediodia',
                  morning: 'de la mañana',
                  afternoon: 'de la tarde',
                  evening: 'de la tarde',
                  night: 'de la noche',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'medianoche',
                  noon: 'mediodia',
                  morning: 'de la mañana',
                  afternoon: 'de la tarde',
                  evening: 'de la tarde',
                  night: 'de la noche',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/match/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchPatternFn/index.js',
            ),
          ),
          _index2 = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchFn/index.js',
            ),
          ),
          _default = {
            ordinalNumber: (0, _index.default)({
              matchPattern: /^(\d+)(º)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index2.default)({
              matchPatterns: {
                narrow: /^(ac|dc|a|d)/i,
                abbreviated: /^(a\.?\s?c\.?|a\.?\s?e\.?\s?c\.?|d\.?\s?c\.?|e\.?\s?c\.?)/i,
                wide: /^(antes de cristo|antes de la era com[uú]n|despu[eé]s de cristo|era com[uú]n)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: [/^ac/i, /^dc/i],
                wide: [
                  /^(antes de cristo|antes de la era com[uú]n)/i,
                  /^(despu[eé]s de cristo|era com[uú]n)/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index2.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^T[1234]/i,
                wide: /^[1234](º)? trimestre/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/1/i, /2/i, /3/i, /4/i] },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index2.default)({
              matchPatterns: {
                narrow: /^[efmajsond]/i,
                abbreviated: /^(ene|feb|mar|abr|may|jun|jul|ago|sep|oct|nov|dic)/i,
                wide: /^(enero|febrero|marzo|abril|mayo|junio|julio|agosto|septiembre|octubre|noviembre|diciembre)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^e/i,
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
                  /^en/i,
                  /^feb/i,
                  /^mar/i,
                  /^abr/i,
                  /^may/i,
                  /^jun/i,
                  /^jul/i,
                  /^ago/i,
                  /^sep/i,
                  /^oct/i,
                  /^nov/i,
                  /^dic/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index2.default)({
              matchPatterns: {
                narrow: /^[dlmjvs]/i,
                short: /^(do|lu|ma|mi|ju|vi|s[áa])/i,
                abbreviated: /^(dom|lun|mar|mi[ée]|jue|vie|s[áa]b)/i,
                wide: /^(domingo|lunes|martes|mi[ée]rcoles|jueves|viernes|s[áa]bado)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^d/i, /^l/i, /^m/i, /^m/i, /^j/i, /^v/i, /^s/i],
                any: [/^do/i, /^lu/i, /^ma/i, /^mi/i, /^ju/i, /^vi/i, /^sa/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index2.default)({
              matchPatterns: {
                narrow: /^(a|p|mn|md|(de la|a las) (mañana|tarde|noche))/i,
                any: /^([ap]\.?\s?m\.?|medianoche|mediodia|(de la|a las) (mañana|tarde|noche))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^a/i,
                  pm: /^p/i,
                  midnight: /^mn/i,
                  noon: /^md/i,
                  morning: /mañana/i,
                  afternoon: /tarde/i,
                  evening: /tarde/i,
                  night: /noche/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'es',
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
