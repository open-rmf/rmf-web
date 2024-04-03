(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [55416, 45585, 69107, 95565, 79489, 44726, 60087, 38509, 75261, 61315],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'ավելի քիչ քան 1 վայրկյան',
              other: 'ավելի քիչ քան {{count}} վայրկյան',
            },
            xSeconds: { one: '1 վայրկյան', other: '{{count}} վայրկյան' },
            halfAMinute: 'կես րոպե',
            lessThanXMinutes: {
              one: 'ավելի քիչ քան 1 րոպե',
              other: 'ավելի քիչ քան {{count}} րոպե',
            },
            xMinutes: { one: '1 րոպե', other: '{{count}} րոպե' },
            aboutXHours: { one: 'մոտ 1 ժամ', other: 'մոտ {{count}} ժամ' },
            xHours: { one: '1 ժամ', other: '{{count}} ժամ' },
            xDays: { one: '1 օր', other: '{{count}} օր' },
            aboutXWeeks: { one: 'մոտ 1 շաբաթ', other: 'մոտ {{count}} շաբաթ' },
            xWeeks: { one: '1 շաբաթ', other: '{{count}} շաբաթ' },
            aboutXMonths: { one: 'մոտ 1 ամիս', other: 'մոտ {{count}} ամիս' },
            xMonths: { one: '1 ամիս', other: '{{count}} ամիս' },
            aboutXYears: { one: 'մոտ 1 տարի', other: 'մոտ {{count}} տարի' },
            xYears: { one: '1 տարի', other: '{{count}} տարի' },
            overXYears: { one: 'ավելի քան 1 տարի', other: 'ավելի քան {{count}} տարի' },
            almostXYears: { one: 'համարյա 1 տարի', other: 'համարյա {{count}} տարի' },
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
                  ? result + ' հետո'
                  : result + ' առաջ'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatLong/index.js':
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
                full: 'd MMMM, y, EEEE',
                long: 'd MMMM, y',
                medium: 'd MMM, y',
                short: 'dd.MM.yyyy',
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
                full: "{{date}} 'ժ․'{{time}}",
                long: "{{date}} 'ժ․'{{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'նախորդ' eeee p'֊ին'",
            yesterday: "'երեկ' p'֊ին'",
            today: "'այսօր' p'֊ին'",
            tomorrow: "'վաղը' p'֊ին'",
            nextWeek: "'հաջորդ' eeee p'֊ին'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/localize/index.js':
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
              return rem100 < 10 && rem100 % 10 == 1 ? number + '֊ին' : number + '֊րդ';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['Ք', 'Մ'],
                abbreviated: ['ՔԱ', 'ՄԹ'],
                wide: ['Քրիստոսից առաջ', 'Մեր թվարկության'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Ք1', 'Ք2', 'Ք3', 'Ք4'],
                wide: ['1֊ին քառորդ', '2֊րդ քառորդ', '3֊րդ քառորդ', '4֊րդ քառորդ'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['Հ', 'Փ', 'Մ', 'Ա', 'Մ', 'Հ', 'Հ', 'Օ', 'Ս', 'Հ', 'Ն', 'Դ'],
                abbreviated: [
                  'հուն',
                  'փետ',
                  'մար',
                  'ապր',
                  'մայ',
                  'հուն',
                  'հուլ',
                  'օգս',
                  'սեպ',
                  'հոկ',
                  'նոյ',
                  'դեկ',
                ],
                wide: [
                  'հունվար',
                  'փետրվար',
                  'մարտ',
                  'ապրիլ',
                  'մայիս',
                  'հունիս',
                  'հուլիս',
                  'օգոստոս',
                  'սեպտեմբեր',
                  'հոկտեմբեր',
                  'նոյեմբեր',
                  'դեկտեմբեր',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Կ', 'Ե', 'Ե', 'Չ', 'Հ', 'Ո', 'Շ'],
                short: ['կր', 'եր', 'եք', 'չք', 'հգ', 'ուր', 'շբ'],
                abbreviated: ['կիր', 'երկ', 'երք', 'չոր', 'հնգ', 'ուրբ', 'շաբ'],
                wide: [
                  'կիրակի',
                  'երկուշաբթի',
                  'երեքշաբթի',
                  'չորեքշաբթի',
                  'հինգշաբթի',
                  'ուրբաթ',
                  'շաբաթ',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'կեսգշ',
                  noon: 'կեսօր',
                  morning: 'առավոտ',
                  afternoon: 'ցերեկ',
                  evening: 'երեկո',
                  night: 'գիշեր',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'կեսգիշեր',
                  noon: 'կեսօր',
                  morning: 'առավոտ',
                  afternoon: 'ցերեկ',
                  evening: 'երեկո',
                  night: 'գիշեր',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'կեսգիշեր',
                  noon: 'կեսօր',
                  morning: 'առավոտ',
                  afternoon: 'ցերեկ',
                  evening: 'երեկո',
                  night: 'գիշեր',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'կեսգշ',
                  noon: 'կեսօր',
                  morning: 'առավոտը',
                  afternoon: 'ցերեկը',
                  evening: 'երեկոյան',
                  night: 'գիշերը',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'կեսգիշերին',
                  noon: 'կեսօրին',
                  morning: 'առավոտը',
                  afternoon: 'ցերեկը',
                  evening: 'երեկոյան',
                  night: 'գիշերը',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'կեսգիշերին',
                  noon: 'կեսօրին',
                  morning: 'առավոտը',
                  afternoon: 'ցերեկը',
                  evening: 'երեկոյան',
                  night: 'գիշերը',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/match/index.js':
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
              matchPattern: /^(\d+)((-|֊)?(ին|րդ))?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(Ք|Մ)/i,
                abbreviated: /^(Ք\.?\s?Ա\.?|Մ\.?\s?Թ\.?\s?Ա\.?|Մ\.?\s?Թ\.?|Ք\.?\s?Հ\.?)/i,
                wide: /^(քրիստոսից առաջ|մեր թվարկությունից առաջ|մեր թվարկության|քրիստոսից հետո)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^ք/i, /^մ/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^ք[1234]/i,
                wide: /^[1234]((-|֊)?(ին|րդ)) քառորդ/i,
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
                narrow: /^[հփմաօսնդ]/i,
                abbreviated: /^(հուն|փետ|մար|ապր|մայ|հուն|հուլ|օգս|սեպ|հոկ|նոյ|դեկ)/i,
                wide: /^(հունվար|փետրվար|մարտ|ապրիլ|մայիս|հունիս|հուլիս|օգոստոս|սեպտեմբեր|հոկտեմբեր|նոյեմբեր|դեկտեմբեր)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^հ/i,
                  /^փ/i,
                  /^մ/i,
                  /^ա/i,
                  /^մ/i,
                  /^հ/i,
                  /^հ/i,
                  /^օ/i,
                  /^ս/i,
                  /^հ/i,
                  /^ն/i,
                  /^դ/i,
                ],
                any: [
                  /^հու/i,
                  /^փ/i,
                  /^մար/i,
                  /^ա/i,
                  /^մայ/i,
                  /^հուն/i,
                  /^հուլ/i,
                  /^օ/i,
                  /^ս/i,
                  /^հոկ/i,
                  /^ն/i,
                  /^դ/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[եչհոշկ]/i,
                short: /^(կր|եր|եք|չք|հգ|ուր|շբ)/i,
                abbreviated: /^(կիր|երկ|երք|չոր|հնգ|ուրբ|շաբ)/i,
                wide: /^(կիրակի|երկուշաբթի|երեքշաբթի|չորեքշաբթի|հինգշաբթի|ուրբաթ|շաբաթ)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^կ/i, /^ե/i, /^ե/i, /^չ/i, /^հ/i, /^(ո|Ո)/, /^շ/i],
                short: [/^կ/i, /^եր/i, /^եք/i, /^չ/i, /^հ/i, /^(ո|Ո)/, /^շ/i],
                abbreviated: [/^կ/i, /^երկ/i, /^երք/i, /^չ/i, /^հ/i, /^(ո|Ո)/, /^շ/i],
                wide: [/^կ/i, /^երկ/i, /^երե/i, /^չ/i, /^հ/i, /^(ո|Ո)/, /^շ/i],
              },
              defaultParseWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^([ap]|կեսգշ|կեսօր|(առավոտը?|ցերեկը?|երեկո(յան)?|գիշերը?))/i,
                any: /^([ap]\.?\s?m\.?|կեսգիշեր(ին)?|կեսօր(ին)?|(առավոտը?|ցերեկը?|երեկո(յան)?|գիշերը?))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^a/i,
                  pm: /^p/i,
                  midnight: /կեսգիշեր/i,
                  noon: /կեսօր/i,
                  morning: /առավոտ/i,
                  afternoon: /ցերեկ/i,
                  evening: /երեկո/i,
                  night: /գիշեր/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'hy',
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
