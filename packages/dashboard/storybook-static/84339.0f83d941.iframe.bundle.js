(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [84339, 45585, 69107, 95565, 79489, 33237, 13888, 42770, 97610, 19662],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'minna en 1 sekúnda', other: 'minna en {{count}} sekúndur' },
            xSeconds: { one: '1 sekúnda', other: '{{count}} sekúndur' },
            halfAMinute: 'hálf mínúta',
            lessThanXMinutes: { one: 'minna en 1 mínúta', other: 'minna en {{count}} mínútur' },
            xMinutes: { one: '1 mínúta', other: '{{count}} mínútur' },
            aboutXHours: { one: 'u.þ.b. 1 klukkustund', other: 'u.þ.b. {{count}} klukkustundir' },
            xHours: { one: '1 klukkustund', other: '{{count}} klukkustundir' },
            xDays: { one: '1 dagur', other: '{{count}} dagar' },
            aboutXWeeks: { one: 'um viku', other: 'um {{count}} vikur' },
            xWeeks: { one: '1 viku', other: '{{count}} vikur' },
            aboutXMonths: { one: 'u.þ.b. 1 mánuður', other: 'u.þ.b. {{count}} mánuðir' },
            xMonths: { one: '1 mánuður', other: '{{count}} mánuðir' },
            aboutXYears: { one: 'u.þ.b. 1 ár', other: 'u.þ.b. {{count}} ár' },
            xYears: { one: '1 ár', other: '{{count}} ár' },
            overXYears: { one: 'meira en 1 ár', other: 'meira en {{count}} ár' },
            almostXYears: { one: 'næstum 1 ár', other: 'næstum {{count}} ár' },
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
                  ? 'í ' + result
                  : result + ' síðan'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatLong/index.js':
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
                full: 'EEEE, do MMMM y',
                long: 'do MMMM y',
                medium: 'do MMM y',
                short: 'd.MM.y',
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'síðasta' dddd 'kl.' p",
            yesterday: "'í gær kl.' p",
            today: "'í dag kl.' p",
            tomorrow: "'á morgun kl.' p",
            nextWeek: "dddd 'kl.' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/localize/index.js':
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
                narrow: ['f.Kr.', 'e.Kr.'],
                abbreviated: ['f.Kr.', 'e.Kr.'],
                wide: ['fyrir Krist', 'eftir Krist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1F', '2F', '3F', '4F'],
                wide: ['1. fjórðungur', '2. fjórðungur', '3. fjórðungur', '4. fjórðungur'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['J', 'F', 'M', 'A', 'M', 'J', 'J', 'Á', 'S', 'Ó', 'N', 'D'],
                abbreviated: [
                  'jan.',
                  'feb.',
                  'mars',
                  'apríl',
                  'maí',
                  'júní',
                  'júlí',
                  'ágúst',
                  'sept.',
                  'okt.',
                  'nóv.',
                  'des.',
                ],
                wide: [
                  'janúar',
                  'febrúar',
                  'mars',
                  'apríl',
                  'maí',
                  'júní',
                  'júlí',
                  'ágúst',
                  'september',
                  'október',
                  'nóvember',
                  'desember',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['S', 'M', 'Þ', 'M', 'F', 'F', 'L'],
                short: ['Su', 'Má', 'Þr', 'Mi', 'Fi', 'Fö', 'La'],
                abbreviated: ['sun.', 'mán.', 'þri.', 'mið.', 'fim.', 'fös.', 'lau.'],
                wide: [
                  'sunnudagur',
                  'mánudagur',
                  'þriðjudagur',
                  'miðvikudagur',
                  'fimmtudagur',
                  'föstudagur',
                  'laugardagur',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'f',
                  pm: 'e',
                  midnight: 'miðnætti',
                  noon: 'hádegi',
                  morning: 'morgunn',
                  afternoon: 'síðdegi',
                  evening: 'kvöld',
                  night: 'nótt',
                },
                abbreviated: {
                  am: 'f.h.',
                  pm: 'e.h.',
                  midnight: 'miðnætti',
                  noon: 'hádegi',
                  morning: 'morgunn',
                  afternoon: 'síðdegi',
                  evening: 'kvöld',
                  night: 'nótt',
                },
                wide: {
                  am: 'fyrir hádegi',
                  pm: 'eftir hádegi',
                  midnight: 'miðnætti',
                  noon: 'hádegi',
                  morning: 'morgunn',
                  afternoon: 'síðdegi',
                  evening: 'kvöld',
                  night: 'nótt',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'f',
                  pm: 'e',
                  midnight: 'á miðnætti',
                  noon: 'á hádegi',
                  morning: 'að morgni',
                  afternoon: 'síðdegis',
                  evening: 'um kvöld',
                  night: 'um nótt',
                },
                abbreviated: {
                  am: 'f.h.',
                  pm: 'e.h.',
                  midnight: 'á miðnætti',
                  noon: 'á hádegi',
                  morning: 'að morgni',
                  afternoon: 'síðdegis',
                  evening: 'um kvöld',
                  night: 'um nótt',
                },
                wide: {
                  am: 'fyrir hádegi',
                  pm: 'eftir hádegi',
                  midnight: 'á miðnætti',
                  noon: 'á hádegi',
                  morning: 'að morgni',
                  afternoon: 'síðdegis',
                  evening: 'um kvöld',
                  night: 'um nótt',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/match/index.js':
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
              matchPattern: /^(\d+)(\.)?/i,
              parsePattern: /\d+(\.)?/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(f\.Kr\.|e\.Kr\.)/i,
                abbreviated: /^(f\.Kr\.|e\.Kr\.)/i,
                wide: /^(fyrir Krist|eftir Krist)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^(f\.Kr\.)/i, /^(e\.Kr\.)/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]\.?/i,
                abbreviated: /^q[1234]\.?/i,
                wide: /^[1234]\.? fjórðungur/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/1\.?/i, /2\.?/i, /3\.?/i, /4\.?/i] },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^[jfmásónd]/i,
                abbreviated:
                  /^(jan\.|feb\.|mars\.|apríl\.|maí|júní|júlí|águst|sep\.|oct\.|nov\.|dec\.)/i,
                wide: /^(januar|febrúar|mars|apríl|maí|júní|júlí|águst|september|október|nóvember|desember)/i,
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
                  /^á/i,
                  /^s/i,
                  /^ó/i,
                  /^n/i,
                  /^d/i,
                ],
                any: [
                  /^ja/i,
                  /^f/i,
                  /^mar/i,
                  /^ap/i,
                  /^maí/i,
                  /^jún/i,
                  /^júl/i,
                  /^áu/i,
                  /^s/i,
                  /^ó/i,
                  /^n/i,
                  /^d/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[smtwf]/i,
                short: /^(su|má|þr|mi|fi|fö|la)/i,
                abbreviated: /^(sun|mán|þri|mið|fim|fös|lau)\.?/i,
                wide: /^(sunnudagur|mánudagur|þriðjudagur|miðvikudagur|fimmtudagur|föstudagur|laugardagur)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^s/i, /^m/i, /^þ/i, /^m/i, /^f/i, /^f/i, /^l/i],
                any: [/^su/i, /^má/i, /^þr/i, /^mi/i, /^fi/i, /^fö/i, /^la/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(f|e|síðdegis|(á|að|um) (morgni|kvöld|nótt|miðnætti))/i,
                any: /^(fyrir hádegi|eftir hádegi|[ef]\.?h\.?|síðdegis|morgunn|(á|að|um) (morgni|kvöld|nótt|miðnætti))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^f/i,
                  pm: /^e/i,
                  midnight: /^mi/i,
                  noon: /^há/i,
                  morning: /morgunn/i,
                  afternoon: /síðdegi/i,
                  evening: /kvöld/i,
                  night: /nótt/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'is',
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
