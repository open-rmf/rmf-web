(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [70926, 45585, 69107, 95565, 79489, 65316, 85589, 14327, 3447, 46297],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        function futureSeconds(text) {
          return text.replace(/sekuntia?/, 'sekunnin');
        }
        function futureMinutes(text) {
          return text.replace(/minuuttia?/, 'minuutin');
        }
        function futureHours(text) {
          return text.replace(/tuntia?/, 'tunnin');
        }
        function futureWeeks(text) {
          return text.replace(/(viikko|viikkoa)/, 'viikon');
        }
        function futureMonths(text) {
          return text.replace(/(kuukausi|kuukautta)/, 'kuukauden');
        }
        function futureYears(text) {
          return text.replace(/(vuosi|vuotta)/, 'vuoden');
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'alle sekunti',
              other: 'alle {{count}} sekuntia',
              futureTense: futureSeconds,
            },
            xSeconds: { one: 'sekunti', other: '{{count}} sekuntia', futureTense: futureSeconds },
            halfAMinute: {
              one: 'puoli minuuttia',
              other: 'puoli minuuttia',
              futureTense: function futureTense(_text) {
                return 'puolen minuutin';
              },
            },
            lessThanXMinutes: {
              one: 'alle minuutti',
              other: 'alle {{count}} minuuttia',
              futureTense: futureMinutes,
            },
            xMinutes: { one: 'minuutti', other: '{{count}} minuuttia', futureTense: futureMinutes },
            aboutXHours: {
              one: 'noin tunti',
              other: 'noin {{count}} tuntia',
              futureTense: futureHours,
            },
            xHours: { one: 'tunti', other: '{{count}} tuntia', futureTense: futureHours },
            xDays: {
              one: 'päivä',
              other: '{{count}} päivää',
              futureTense: function futureDays(text) {
                return text.replace(/päivää?/, 'päivän');
              },
            },
            aboutXWeeks: {
              one: 'noin viikko',
              other: 'noin {{count}} viikkoa',
              futureTense: futureWeeks,
            },
            xWeeks: { one: 'viikko', other: '{{count}} viikkoa', futureTense: futureWeeks },
            aboutXMonths: {
              one: 'noin kuukausi',
              other: 'noin {{count}} kuukautta',
              futureTense: futureMonths,
            },
            xMonths: { one: 'kuukausi', other: '{{count}} kuukautta', futureTense: futureMonths },
            aboutXYears: {
              one: 'noin vuosi',
              other: 'noin {{count}} vuotta',
              futureTense: futureYears,
            },
            xYears: { one: 'vuosi', other: '{{count}} vuotta', futureTense: futureYears },
            overXYears: {
              one: 'yli vuosi',
              other: 'yli {{count}} vuotta',
              futureTense: futureYears,
            },
            almostXYears: {
              one: 'lähes vuosi',
              other: 'lähes {{count}} vuotta',
              futureTense: futureYears,
            },
          },
          _default = function formatDistance(token, count, options) {
            var tokenValue = formatDistanceLocale[token],
              result =
                1 === count ? tokenValue.one : tokenValue.other.replace('{{count}}', String(count));
            return null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? tokenValue.futureTense(result) + ' kuluttua'
                : result + ' sitten'
              : result;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatLong/index.js':
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
                full: 'eeee d. MMMM y',
                long: 'd. MMMM y',
                medium: 'd. MMM y',
                short: 'd.M.y',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'HH.mm.ss zzzz',
                long: 'HH.mm.ss z',
                medium: 'HH.mm.ss',
                short: 'HH.mm',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: "{{date}} 'klo' {{time}}",
                long: "{{date}} 'klo' {{time}}",
                medium: '{{date}} {{time}}',
                short: '{{date}} {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'viime' eeee 'klo' p",
            yesterday: "'eilen klo' p",
            today: "'tänään klo' p",
            tomorrow: "'huomenna klo' p",
            nextWeek: "'ensi' eeee 'klo' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/localize/index.js':
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
          monthValues = {
            narrow: ['T', 'H', 'M', 'H', 'T', 'K', 'H', 'E', 'S', 'L', 'M', 'J'],
            abbreviated: [
              'tammi',
              'helmi',
              'maalis',
              'huhti',
              'touko',
              'kesä',
              'heinä',
              'elo',
              'syys',
              'loka',
              'marras',
              'joulu',
            ],
            wide: [
              'tammikuu',
              'helmikuu',
              'maaliskuu',
              'huhtikuu',
              'toukokuu',
              'kesäkuu',
              'heinäkuu',
              'elokuu',
              'syyskuu',
              'lokakuu',
              'marraskuu',
              'joulukuu',
            ],
          },
          formattingMonthValues = {
            narrow: monthValues.narrow,
            abbreviated: monthValues.abbreviated,
            wide: [
              'tammikuuta',
              'helmikuuta',
              'maaliskuuta',
              'huhtikuuta',
              'toukokuuta',
              'kesäkuuta',
              'heinäkuuta',
              'elokuuta',
              'syyskuuta',
              'lokakuuta',
              'marraskuuta',
              'joulukuuta',
            ],
          },
          dayValues = {
            narrow: ['S', 'M', 'T', 'K', 'T', 'P', 'L'],
            short: ['su', 'ma', 'ti', 'ke', 'to', 'pe', 'la'],
            abbreviated: ['sunn.', 'maan.', 'tiis.', 'kesk.', 'torst.', 'perj.', 'la'],
            wide: [
              'sunnuntai',
              'maanantai',
              'tiistai',
              'keskiviikko',
              'torstai',
              'perjantai',
              'lauantai',
            ],
          },
          formattingDayValues = {
            narrow: dayValues.narrow,
            short: dayValues.short,
            abbreviated: dayValues.abbreviated,
            wide: [
              'sunnuntaina',
              'maanantaina',
              'tiistaina',
              'keskiviikkona',
              'torstaina',
              'perjantaina',
              'lauantaina',
            ],
          },
          _default = {
            ordinalNumber: function ordinalNumber(dirtyNumber, _options) {
              return Number(dirtyNumber) + '.';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['eaa.', 'jaa.'],
                abbreviated: ['eaa.', 'jaa.'],
                wide: ['ennen ajanlaskun alkua', 'jälkeen ajanlaskun alun'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1. kvartaali', '2. kvartaali', '3. kvartaali', '4. kvartaali'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: monthValues,
              defaultWidth: 'wide',
              formattingValues: formattingMonthValues,
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: dayValues,
              defaultWidth: 'wide',
              formattingValues: formattingDayValues,
              defaultFormattingWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ap',
                  pm: 'ip',
                  midnight: 'keskiyö',
                  noon: 'keskipäivä',
                  morning: 'ap',
                  afternoon: 'ip',
                  evening: 'illalla',
                  night: 'yöllä',
                },
                abbreviated: {
                  am: 'ap',
                  pm: 'ip',
                  midnight: 'keskiyö',
                  noon: 'keskipäivä',
                  morning: 'ap',
                  afternoon: 'ip',
                  evening: 'illalla',
                  night: 'yöllä',
                },
                wide: {
                  am: 'ap',
                  pm: 'ip',
                  midnight: 'keskiyöllä',
                  noon: 'keskipäivällä',
                  morning: 'aamupäivällä',
                  afternoon: 'iltapäivällä',
                  evening: 'illalla',
                  night: 'yöllä',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/match/index.js':
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
              matchPattern: /^(\d+)(\.)/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(e|j)/i,
                abbreviated: /^(eaa.|jaa.)/i,
                wide: /^(ennen ajanlaskun alkua|jälkeen ajanlaskun alun)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^e/i, /^j/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^q[1234]/i,
                wide: /^[1234]\.? kvartaali/i,
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
                narrow: /^[thmkeslj]/i,
                abbreviated:
                  /^(tammi|helmi|maalis|huhti|touko|kesä|heinä|elo|syys|loka|marras|joulu)/i,
                wide: /^(tammikuu|helmikuu|maaliskuu|huhtikuu|toukokuu|kesäkuu|heinäkuu|elokuu|syyskuu|lokakuu|marraskuu|joulukuu)(ta)?/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^t/i,
                  /^h/i,
                  /^m/i,
                  /^h/i,
                  /^t/i,
                  /^k/i,
                  /^h/i,
                  /^e/i,
                  /^s/i,
                  /^l/i,
                  /^m/i,
                  /^j/i,
                ],
                any: [
                  /^ta/i,
                  /^hel/i,
                  /^maa/i,
                  /^hu/i,
                  /^to/i,
                  /^k/i,
                  /^hei/i,
                  /^e/i,
                  /^s/i,
                  /^l/i,
                  /^mar/i,
                  /^j/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[smtkpl]/i,
                short: /^(su|ma|ti|ke|to|pe|la)/i,
                abbreviated: /^(sunn.|maan.|tiis.|kesk.|torst.|perj.|la)/i,
                wide: /^(sunnuntai|maanantai|tiistai|keskiviikko|torstai|perjantai|lauantai)(na)?/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^s/i, /^m/i, /^t/i, /^k/i, /^t/i, /^p/i, /^l/i],
                any: [/^s/i, /^m/i, /^ti/i, /^k/i, /^to/i, /^p/i, /^l/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ap|ip|keskiyö|keskipäivä|aamupäivällä|iltapäivällä|illalla|yöllä)/i,
                any: /^(ap|ip|keskiyöllä|keskipäivällä|aamupäivällä|iltapäivällä|illalla|yöllä)/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^ap/i,
                  pm: /^ip/i,
                  midnight: /^keskiyö/i,
                  noon: /^keskipäivä/i,
                  morning: /aamupäivällä/i,
                  afternoon: /iltapäivällä/i,
                  evening: /illalla/i,
                  night: /yöllä/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fi/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'fi',
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
