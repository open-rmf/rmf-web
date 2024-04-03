(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [76910, 45585, 69107, 95565, 79489, 8036, 5845, 66423, 19607, 39001],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'menos de um segundo', other: 'menos de {{count}} segundos' },
            xSeconds: { one: '1 segundo', other: '{{count}} segundos' },
            halfAMinute: 'meio minuto',
            lessThanXMinutes: { one: 'menos de um minuto', other: 'menos de {{count}} minutos' },
            xMinutes: { one: '1 minuto', other: '{{count}} minutos' },
            aboutXHours: { one: 'cerca de 1 hora', other: 'cerca de {{count}} horas' },
            xHours: { one: '1 hora', other: '{{count}} horas' },
            xDays: { one: '1 dia', other: '{{count}} dias' },
            aboutXWeeks: { one: 'cerca de 1 semana', other: 'cerca de {{count}} semanas' },
            xWeeks: { one: '1 semana', other: '{{count}} semanas' },
            aboutXMonths: { one: 'cerca de 1 mês', other: 'cerca de {{count}} meses' },
            xMonths: { one: '1 mês', other: '{{count}} meses' },
            aboutXYears: { one: 'cerca de 1 ano', other: 'cerca de {{count}} anos' },
            xYears: { one: '1 ano', other: '{{count}} anos' },
            overXYears: { one: 'mais de 1 ano', other: 'mais de {{count}} anos' },
            almostXYears: { one: 'quase 1 ano', other: 'quase {{count}} anos' },
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
                  ? 'em ' + result
                  : 'há ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatLong/index.js':
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
                short: 'dd/MM/yyyy',
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
                full: "{{date}} 'às' {{time}}",
                long: "{{date}} 'às' {{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date) {
              var weekday = date.getUTCDay();
              return "'" + (0 === weekday || 6 === weekday ? 'último' : 'última') + "' eeee 'às' p";
            },
            yesterday: "'ontem às' p",
            today: "'hoje às' p",
            tomorrow: "'amanhã às' p",
            nextWeek: "eeee 'às' p",
            other: 'P',
          },
          _default = function formatRelative(token, date, _baseDate, _options) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/localize/index.js':
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
              var number = Number(dirtyNumber);
              return 'week' === (null == options ? void 0 : options.unit)
                ? number + 'ª'
                : number + 'º';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['AC', 'DC'],
                abbreviated: ['AC', 'DC'],
                wide: ['antes de cristo', 'depois de cristo'],
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
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['j', 'f', 'm', 'a', 'm', 'j', 'j', 'a', 's', 'o', 'n', 'd'],
                abbreviated: [
                  'jan',
                  'fev',
                  'mar',
                  'abr',
                  'mai',
                  'jun',
                  'jul',
                  'ago',
                  'set',
                  'out',
                  'nov',
                  'dez',
                ],
                wide: [
                  'janeiro',
                  'fevereiro',
                  'março',
                  'abril',
                  'maio',
                  'junho',
                  'julho',
                  'agosto',
                  'setembro',
                  'outubro',
                  'novembro',
                  'dezembro',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['D', 'S', 'T', 'Q', 'Q', 'S', 'S'],
                short: ['dom', 'seg', 'ter', 'qua', 'qui', 'sex', 'sab'],
                abbreviated: ['domingo', 'segunda', 'terça', 'quarta', 'quinta', 'sexta', 'sábado'],
                wide: [
                  'domingo',
                  'segunda-feira',
                  'terça-feira',
                  'quarta-feira',
                  'quinta-feira',
                  'sexta-feira',
                  'sábado',
                ],
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
                  morning: 'manhã',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noite',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'manhã',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noite',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'manhã',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noite',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'mn',
                  noon: 'md',
                  morning: 'da manhã',
                  afternoon: 'da tarde',
                  evening: 'da tarde',
                  night: 'da noite',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'da manhã',
                  afternoon: 'da tarde',
                  evening: 'da tarde',
                  night: 'da noite',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'da manhã',
                  afternoon: 'da tarde',
                  evening: 'da tarde',
                  night: 'da noite',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/match/index.js':
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
              matchPattern: /^(\d+)[ºªo]?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ac|dc|a|d)/i,
                abbreviated: /^(a\.?\s?c\.?|d\.?\s?c\.?)/i,
                wide: /^(antes de cristo|depois de cristo)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: [/^ac/i, /^dc/i],
                wide: [/^antes de cristo/i, /^depois de cristo/i],
              },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
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
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^[jfmajsond]/i,
                abbreviated: /^(jan|fev|mar|abr|mai|jun|jul|ago|set|out|nov|dez)/i,
                wide: /^(janeiro|fevereiro|março|abril|maio|junho|julho|agosto|setembro|outubro|novembro|dezembro)/i,
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
                  /^fev/i,
                  /^mar/i,
                  /^abr/i,
                  /^mai/i,
                  /^jun/i,
                  /^jul/i,
                  /^ago/i,
                  /^set/i,
                  /^out/i,
                  /^nov/i,
                  /^dez/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(dom|[23456]ª?|s[aá]b)/i,
                short: /^(dom|[23456]ª?|s[aá]b)/i,
                abbreviated: /^(dom|seg|ter|qua|qui|sex|s[aá]b)/i,
                wide: /^(domingo|(segunda|ter[cç]a|quarta|quinta|sexta)([- ]feira)?|s[aá]bado)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                short: [/^d/i, /^2/i, /^3/i, /^4/i, /^5/i, /^6/i, /^s[aá]/i],
                narrow: [/^d/i, /^2/i, /^3/i, /^4/i, /^5/i, /^6/i, /^s[aá]/i],
                any: [/^d/i, /^seg/i, /^t/i, /^qua/i, /^qui/i, /^sex/i, /^s[aá]b/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(a|p|mn|md|(da) (manhã|tarde|noite))/i,
                any: /^([ap]\.?\s?m\.?|meia[-\s]noite|meio[-\s]dia|(da) (manhã|tarde|noite))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^a/i,
                  pm: /^p/i,
                  midnight: /^mn|^meia[-\s]noite/i,
                  noon: /^md|^meio[-\s]dia/i,
                  morning: /manhã/i,
                  afternoon: /tarde/i,
                  evening: /tarde/i,
                  night: /noite/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'pt-BR',
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
