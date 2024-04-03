(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [27023, 45585, 69107, 95565, 79489, 15617, 24172, 75430, 90262, 54898],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'હમણાં', other: '​આશરે {{count}} સેકંડ' },
            xSeconds: { one: '1 સેકંડ', other: '{{count}} સેકંડ' },
            halfAMinute: 'અડધી મિનિટ',
            lessThanXMinutes: { one: 'આ મિનિટ', other: '​આશરે {{count}} મિનિટ' },
            xMinutes: { one: '1 મિનિટ', other: '{{count}} મિનિટ' },
            aboutXHours: { one: '​આશરે 1 કલાક', other: '​આશરે {{count}} કલાક' },
            xHours: { one: '1 કલાક', other: '{{count}} કલાક' },
            xDays: { one: '1 દિવસ', other: '{{count}} દિવસ' },
            aboutXWeeks: { one: 'આશરે 1 અઠવાડિયું', other: 'આશરે {{count}} અઠવાડિયા' },
            xWeeks: { one: '1 અઠવાડિયું', other: '{{count}} અઠવાડિયા' },
            aboutXMonths: { one: 'આશરે 1 મહિનો', other: 'આશરે {{count}} મહિના' },
            xMonths: { one: '1 મહિનો', other: '{{count}} મહિના' },
            aboutXYears: { one: 'આશરે 1 વર્ષ', other: 'આશરે {{count}} વર્ષ' },
            xYears: { one: '1 વર્ષ', other: '{{count}} વર્ષ' },
            overXYears: { one: '1 વર્ષથી વધુ', other: '{{count}} વર્ષથી વધુ' },
            almostXYears: { one: 'લગભગ 1 વર્ષ', other: 'લગભગ {{count}} વર્ષ' },
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
                  ? result + 'માં'
                  : result + ' પહેલાં'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatLong/index.js':
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
                full: 'EEEE, d MMMM, y',
                long: 'd MMMM, y',
                medium: 'd MMM, y',
                short: 'd/M/yy',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'hh:mm:ss a zzzz',
                long: 'hh:mm:ss a z',
                medium: 'hh:mm:ss a',
                short: 'hh:mm a',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: '{{date}} {{time}}',
                long: '{{date}} {{time}}',
                medium: '{{date}} {{time}}',
                short: '{{date}} {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'પાછલા' eeee p",
            yesterday: "'ગઈકાલે' p",
            today: "'આજે' p",
            tomorrow: "'આવતીકાલે' p",
            nextWeek: 'eeee p',
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/localize/index.js':
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
                narrow: ['ઈસપૂ', 'ઈસ'],
                abbreviated: ['ઈ.સ.પૂર્વે', 'ઈ.સ.'],
                wide: ['ઈસવીસન પૂર્વે', 'ઈસવીસન'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1લો ત્રિમાસ', '2જો ત્રિમાસ', '3જો ત્રિમાસ', '4થો ત્રિમાસ'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['જા', 'ફે', 'મા', 'એ', 'મે', 'જૂ', 'જુ', 'ઓ', 'સ', 'ઓ', 'ન', 'ડિ'],
                abbreviated: [
                  'જાન્યુ',
                  'ફેબ્રુ',
                  'માર્ચ',
                  'એપ્રિલ',
                  'મે',
                  'જૂન',
                  'જુલાઈ',
                  'ઑગસ્ટ',
                  'સપ્ટે',
                  'ઓક્ટો',
                  'નવે',
                  'ડિસે',
                ],
                wide: [
                  'જાન્યુઆરી',
                  'ફેબ્રુઆરી',
                  'માર્ચ',
                  'એપ્રિલ',
                  'મે',
                  'જૂન',
                  'જુલાઇ',
                  'ઓગસ્ટ',
                  'સપ્ટેમ્બર',
                  'ઓક્ટોબર',
                  'નવેમ્બર',
                  'ડિસેમ્બર',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ર', 'સો', 'મં', 'બુ', 'ગુ', 'શુ', 'શ'],
                short: ['ર', 'સો', 'મં', 'બુ', 'ગુ', 'શુ', 'શ'],
                abbreviated: ['રવિ', 'સોમ', 'મંગળ', 'બુધ', 'ગુરુ', 'શુક્ર', 'શનિ'],
                wide: ['રવિવાર', 'સોમવાર', 'મંગળવાર', 'બુધવાર', 'ગુરુવાર', 'શુક્રવાર', 'શનિવાર'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'મ.રાત્રિ',
                  noon: 'બ.',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: '​મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: '​મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'મ.રાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: '​મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/match/index.js':
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
              matchPattern: /^(\d+)(લ|જ|થ|ઠ્ઠ|મ)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ઈસપૂ|ઈસ)/i,
                abbreviated: /^(ઈ\.સ\.પૂર્વે|ઈ\.સ\.)/i,
                wide: /^(ઈસવીસન\sપૂર્વે|ઈસવીસન)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^ઈસપૂ/i, /^ઈસ/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^q[1234]/i,
                wide: /^[1234](લો|જો|થો)? ત્રિમાસ/i,
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
                narrow: /^[જાફેમાએમેજૂજુઓસઓનડિ]/i,
                abbreviated:
                  /^(જાન્યુ|ફેબ્રુ|માર્ચ|એપ્રિલ|મે|જૂન|જુલાઈ|ઑગસ્ટ|સપ્ટે|ઓક્ટો|નવે|ડિસે)/i,
                wide: /^(જાન્યુઆરી|ફેબ્રુઆરી|માર્ચ|એપ્રિલ|મે|જૂન|જુલાઇ|ઓગસ્ટ|સપ્ટેમ્બર|ઓક્ટોબર|નવેમ્બર|ડિસેમ્બર)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^જા/i,
                  /^ફે/i,
                  /^મા/i,
                  /^એ/i,
                  /^મે/i,
                  /^જૂ/i,
                  /^જુ/i,
                  /^ઑગ/i,
                  /^સ/i,
                  /^ઓક્ટો/i,
                  /^ન/i,
                  /^ડિ/i,
                ],
                any: [
                  /^જા/i,
                  /^ફે/i,
                  /^મા/i,
                  /^એ/i,
                  /^મે/i,
                  /^જૂ/i,
                  /^જુ/i,
                  /^ઑગ/i,
                  /^સ/i,
                  /^ઓક્ટો/i,
                  /^ન/i,
                  /^ડિ/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ર|સો|મં|બુ|ગુ|શુ|શ)/i,
                short: /^(ર|સો|મં|બુ|ગુ|શુ|શ)/i,
                abbreviated: /^(રવિ|સોમ|મંગળ|બુધ|ગુરુ|શુક્ર|શનિ)/i,
                wide: /^(રવિવાર|સોમવાર|મંગળવાર|બુધવાર|ગુરુવાર|શુક્રવાર|શનિવાર)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^ર/i, /^સો/i, /^મં/i, /^બુ/i, /^ગુ/i, /^શુ/i, /^શ/i],
                any: [/^ર/i, /^સો/i, /^મં/i, /^બુ/i, /^ગુ/i, /^શુ/i, /^શ/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: { narrow: /^(a|p|મ\.?|સ|બ|સાં|રા)/i, any: /^(a|p|મ\.?|સ|બ|સાં|રા)/i },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^a/i,
                  pm: /^p/i,
                  midnight: /^મ\.?/i,
                  noon: /^બ/i,
                  morning: /સ/i,
                  afternoon: /બ/i,
                  evening: /સાં/i,
                  night: /રા/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'gu',
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
