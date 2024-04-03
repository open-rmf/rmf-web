(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [50932, 45585, 69107, 95565, 79489, 44914, 26243, 49041, 1369, 87415],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
          lessThanXSeconds: {
            present: {
              one: 'manj kot {{count}} sekunda',
              two: 'manj kot {{count}} sekundi',
              few: 'manj kot {{count}} sekunde',
              other: 'manj kot {{count}} sekund',
            },
            past: {
              one: 'manj kot {{count}} sekundo',
              two: 'manj kot {{count}} sekundama',
              few: 'manj kot {{count}} sekundami',
              other: 'manj kot {{count}} sekundami',
            },
            future: {
              one: 'manj kot {{count}} sekundo',
              two: 'manj kot {{count}} sekundi',
              few: 'manj kot {{count}} sekunde',
              other: 'manj kot {{count}} sekund',
            },
          },
          xSeconds: {
            present: {
              one: '{{count}} sekunda',
              two: '{{count}} sekundi',
              few: '{{count}} sekunde',
              other: '{{count}} sekund',
            },
            past: {
              one: '{{count}} sekundo',
              two: '{{count}} sekundama',
              few: '{{count}} sekundami',
              other: '{{count}} sekundami',
            },
            future: {
              one: '{{count}} sekundo',
              two: '{{count}} sekundi',
              few: '{{count}} sekunde',
              other: '{{count}} sekund',
            },
          },
          halfAMinute: 'pol minute',
          lessThanXMinutes: {
            present: {
              one: 'manj kot {{count}} minuta',
              two: 'manj kot {{count}} minuti',
              few: 'manj kot {{count}} minute',
              other: 'manj kot {{count}} minut',
            },
            past: {
              one: 'manj kot {{count}} minuto',
              two: 'manj kot {{count}} minutama',
              few: 'manj kot {{count}} minutami',
              other: 'manj kot {{count}} minutami',
            },
            future: {
              one: 'manj kot {{count}} minuto',
              two: 'manj kot {{count}} minuti',
              few: 'manj kot {{count}} minute',
              other: 'manj kot {{count}} minut',
            },
          },
          xMinutes: {
            present: {
              one: '{{count}} minuta',
              two: '{{count}} minuti',
              few: '{{count}} minute',
              other: '{{count}} minut',
            },
            past: {
              one: '{{count}} minuto',
              two: '{{count}} minutama',
              few: '{{count}} minutami',
              other: '{{count}} minutami',
            },
            future: {
              one: '{{count}} minuto',
              two: '{{count}} minuti',
              few: '{{count}} minute',
              other: '{{count}} minut',
            },
          },
          aboutXHours: {
            present: {
              one: 'približno {{count}} ura',
              two: 'približno {{count}} uri',
              few: 'približno {{count}} ure',
              other: 'približno {{count}} ur',
            },
            past: {
              one: 'približno {{count}} uro',
              two: 'približno {{count}} urama',
              few: 'približno {{count}} urami',
              other: 'približno {{count}} urami',
            },
            future: {
              one: 'približno {{count}} uro',
              two: 'približno {{count}} uri',
              few: 'približno {{count}} ure',
              other: 'približno {{count}} ur',
            },
          },
          xHours: {
            present: {
              one: '{{count}} ura',
              two: '{{count}} uri',
              few: '{{count}} ure',
              other: '{{count}} ur',
            },
            past: {
              one: '{{count}} uro',
              two: '{{count}} urama',
              few: '{{count}} urami',
              other: '{{count}} urami',
            },
            future: {
              one: '{{count}} uro',
              two: '{{count}} uri',
              few: '{{count}} ure',
              other: '{{count}} ur',
            },
          },
          xDays: {
            present: {
              one: '{{count}} dan',
              two: '{{count}} dni',
              few: '{{count}} dni',
              other: '{{count}} dni',
            },
            past: {
              one: '{{count}} dnem',
              two: '{{count}} dnevoma',
              few: '{{count}} dnevi',
              other: '{{count}} dnevi',
            },
            future: {
              one: '{{count}} dan',
              two: '{{count}} dni',
              few: '{{count}} dni',
              other: '{{count}} dni',
            },
          },
          aboutXWeeks: {
            one: 'približno {{count}} teden',
            two: 'približno {{count}} tedna',
            few: 'približno {{count}} tedne',
            other: 'približno {{count}} tednov',
          },
          xWeeks: {
            one: '{{count}} teden',
            two: '{{count}} tedna',
            few: '{{count}} tedne',
            other: '{{count}} tednov',
          },
          aboutXMonths: {
            present: {
              one: 'približno {{count}} mesec',
              two: 'približno {{count}} meseca',
              few: 'približno {{count}} mesece',
              other: 'približno {{count}} mesecev',
            },
            past: {
              one: 'približno {{count}} mesecem',
              two: 'približno {{count}} mesecema',
              few: 'približno {{count}} meseci',
              other: 'približno {{count}} meseci',
            },
            future: {
              one: 'približno {{count}} mesec',
              two: 'približno {{count}} meseca',
              few: 'približno {{count}} mesece',
              other: 'približno {{count}} mesecev',
            },
          },
          xMonths: {
            present: {
              one: '{{count}} mesec',
              two: '{{count}} meseca',
              few: '{{count}} meseci',
              other: '{{count}} mesecev',
            },
            past: {
              one: '{{count}} mesecem',
              two: '{{count}} mesecema',
              few: '{{count}} meseci',
              other: '{{count}} meseci',
            },
            future: {
              one: '{{count}} mesec',
              two: '{{count}} meseca',
              few: '{{count}} mesece',
              other: '{{count}} mesecev',
            },
          },
          aboutXYears: {
            present: {
              one: 'približno {{count}} leto',
              two: 'približno {{count}} leti',
              few: 'približno {{count}} leta',
              other: 'približno {{count}} let',
            },
            past: {
              one: 'približno {{count}} letom',
              two: 'približno {{count}} letoma',
              few: 'približno {{count}} leti',
              other: 'približno {{count}} leti',
            },
            future: {
              one: 'približno {{count}} leto',
              two: 'približno {{count}} leti',
              few: 'približno {{count}} leta',
              other: 'približno {{count}} let',
            },
          },
          xYears: {
            present: {
              one: '{{count}} leto',
              two: '{{count}} leti',
              few: '{{count}} leta',
              other: '{{count}} let',
            },
            past: {
              one: '{{count}} letom',
              two: '{{count}} letoma',
              few: '{{count}} leti',
              other: '{{count}} leti',
            },
            future: {
              one: '{{count}} leto',
              two: '{{count}} leti',
              few: '{{count}} leta',
              other: '{{count}} let',
            },
          },
          overXYears: {
            present: {
              one: 'več kot {{count}} leto',
              two: 'več kot {{count}} leti',
              few: 'več kot {{count}} leta',
              other: 'več kot {{count}} let',
            },
            past: {
              one: 'več kot {{count}} letom',
              two: 'več kot {{count}} letoma',
              few: 'več kot {{count}} leti',
              other: 'več kot {{count}} leti',
            },
            future: {
              one: 'več kot {{count}} leto',
              two: 'več kot {{count}} leti',
              few: 'več kot {{count}} leta',
              other: 'več kot {{count}} let',
            },
          },
          almostXYears: {
            present: {
              one: 'skoraj {{count}} leto',
              two: 'skoraj {{count}} leti',
              few: 'skoraj {{count}} leta',
              other: 'skoraj {{count}} let',
            },
            past: {
              one: 'skoraj {{count}} letom',
              two: 'skoraj {{count}} letoma',
              few: 'skoraj {{count}} leti',
              other: 'skoraj {{count}} leti',
            },
            future: {
              one: 'skoraj {{count}} leto',
              two: 'skoraj {{count}} leti',
              few: 'skoraj {{count}} leta',
              other: 'skoraj {{count}} let',
            },
          },
        };
        var _default = function formatDistance(token, count, options) {
          var result = '',
            tense = 'present';
          null != options &&
            options.addSuffix &&
            (options.comparison && options.comparison > 0
              ? ((tense = 'future'), (result = 'čez '))
              : ((tense = 'past'), (result = 'pred ')));
          var tokenValue = formatDistanceLocale[token];
          if ('string' == typeof tokenValue) result += tokenValue;
          else {
            var form = (function getFormFromCount(count) {
              switch (count % 100) {
                case 1:
                  return 'one';
                case 2:
                  return 'two';
                case 3:
                case 4:
                  return 'few';
                default:
                  return 'other';
              }
            })(count);
            !(function isPluralType(val) {
              return void 0 !== val.one;
            })(tokenValue)
              ? (result += tokenValue[tense][form].replace('{{count}}', String(count)))
              : (result += tokenValue[form].replace('{{count}}', String(count)));
          }
          return result;
        };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatLong/index.js':
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
                full: 'EEEE, dd. MMMM y',
                long: 'dd. MMMM y',
                medium: 'd. MMM y',
                short: 'd. MM. yy',
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date) {
              switch (date.getUTCDay()) {
                case 0:
                  return "'prejšnjo nedeljo ob' p";
                case 3:
                  return "'prejšnjo sredo ob' p";
                case 6:
                  return "'prejšnjo soboto ob' p";
                default:
                  return "'prejšnji' EEEE 'ob' p";
              }
            },
            yesterday: "'včeraj ob' p",
            today: "'danes ob' p",
            tomorrow: "'jutri ob' p",
            nextWeek: function nextWeek(date) {
              switch (date.getUTCDay()) {
                case 0:
                  return "'naslednjo nedeljo ob' p";
                case 3:
                  return "'naslednjo sredo ob' p";
                case 6:
                  return "'naslednjo soboto ob' p";
                default:
                  return "'naslednji' EEEE 'ob' p";
              }
            },
            other: 'P',
          },
          _default = function formatRelative(token, date, _baseDate, _options) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/localize/index.js':
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
                narrow: ['pr. n. št.', 'po n. št.'],
                abbreviated: ['pr. n. št.', 'po n. št.'],
                wide: ['pred našim štetjem', 'po našem štetju'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1. čet.', '2. čet.', '3. čet.', '4. čet.'],
                wide: ['1. četrtletje', '2. četrtletje', '3. četrtletje', '4. četrtletje'],
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
                  'jan.',
                  'feb.',
                  'mar.',
                  'apr.',
                  'maj',
                  'jun.',
                  'jul.',
                  'avg.',
                  'sep.',
                  'okt.',
                  'nov.',
                  'dec.',
                ],
                wide: [
                  'januar',
                  'februar',
                  'marec',
                  'april',
                  'maj',
                  'junij',
                  'julij',
                  'avgust',
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
                narrow: ['n', 'p', 't', 's', 'č', 'p', 's'],
                short: ['ned.', 'pon.', 'tor.', 'sre.', 'čet.', 'pet.', 'sob.'],
                abbreviated: ['ned.', 'pon.', 'tor.', 'sre.', 'čet.', 'pet.', 'sob.'],
                wide: ['nedelja', 'ponedeljek', 'torek', 'sreda', 'četrtek', 'petek', 'sobota'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'd',
                  pm: 'p',
                  midnight: '24.00',
                  noon: '12.00',
                  morning: 'j',
                  afternoon: 'p',
                  evening: 'v',
                  night: 'n',
                },
                abbreviated: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'poln.',
                  noon: 'pold.',
                  morning: 'jut.',
                  afternoon: 'pop.',
                  evening: 'več.',
                  night: 'noč',
                },
                wide: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'polnoč',
                  noon: 'poldne',
                  morning: 'jutro',
                  afternoon: 'popoldne',
                  evening: 'večer',
                  night: 'noč',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'd',
                  pm: 'p',
                  midnight: '24.00',
                  noon: '12.00',
                  morning: 'zj',
                  afternoon: 'p',
                  evening: 'zv',
                  night: 'po',
                },
                abbreviated: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'opoln.',
                  noon: 'opold.',
                  morning: 'zjut.',
                  afternoon: 'pop.',
                  evening: 'zveč.',
                  night: 'ponoči',
                },
                wide: {
                  am: 'dop.',
                  pm: 'pop.',
                  midnight: 'opolnoči',
                  noon: 'opoldne',
                  morning: 'zjutraj',
                  afternoon: 'popoldan',
                  evening: 'zvečer',
                  night: 'ponoči',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/match/index.js':
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
              matchPattern: /^(\d+)\./i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                abbreviated: /^(pr\. n\. št\.|po n\. št\.)/i,
                wide: /^(pred Kristusom|pred na[sš]im [sš]tetjem|po Kristusu|po na[sš]em [sš]tetju|na[sš]ega [sš]tetja)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^pr/i, /^(po|na[sš]em)/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234]\.\s?[čc]et\.?/i,
                wide: /^[1234]\. [čc]etrtletje/i,
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
                abbreviated:
                  /^(jan\.|feb\.|mar\.|apr\.|maj|jun\.|jul\.|avg\.|sep\.|okt\.|nov\.|dec\.)/i,
                wide: /^(januar|februar|marec|april|maj|junij|julij|avgust|september|oktober|november|december)/i,
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
                abbreviated: [
                  /^ja/i,
                  /^fe/i,
                  /^mar/i,
                  /^ap/i,
                  /^maj/i,
                  /^jun/i,
                  /^jul/i,
                  /^av/i,
                  /^s/i,
                  /^o/i,
                  /^n/i,
                  /^d/i,
                ],
                wide: [
                  /^ja/i,
                  /^fe/i,
                  /^mar/i,
                  /^ap/i,
                  /^maj/i,
                  /^jun/i,
                  /^jul/i,
                  /^av/i,
                  /^s/i,
                  /^o/i,
                  /^n/i,
                  /^d/i,
                ],
              },
              defaultParseWidth: 'wide',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[nptsčc]/i,
                short: /^(ned\.|pon\.|tor\.|sre\.|[cč]et\.|pet\.|sob\.)/i,
                abbreviated: /^(ned\.|pon\.|tor\.|sre\.|[cč]et\.|pet\.|sob\.)/i,
                wide: /^(nedelja|ponedeljek|torek|sreda|[cč]etrtek|petek|sobota)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^n/i, /^p/i, /^t/i, /^s/i, /^[cč]/i, /^p/i, /^s/i],
                any: [/^n/i, /^po/i, /^t/i, /^sr/i, /^[cč]/i, /^pe/i, /^so/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(d|po?|z?v|n|z?j|24\.00|12\.00)/i,
                any: /^(dop\.|pop\.|o?poln(\.|o[cč]i?)|o?pold(\.|ne)|z?ve[cč](\.|er)|(po)?no[cč]i?|popold(ne|an)|jut(\.|ro)|zjut(\.|raj))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                narrow: {
                  am: /^d/i,
                  pm: /^p/i,
                  midnight: /^24/i,
                  noon: /^12/i,
                  morning: /^(z?j)/i,
                  afternoon: /^p/i,
                  evening: /^(z?v)/i,
                  night: /^(n|po)/i,
                },
                any: {
                  am: /^dop\./i,
                  pm: /^pop\./i,
                  midnight: /^o?poln/i,
                  noon: /^o?pold/i,
                  morning: /j/i,
                  afternoon: /^pop\./i,
                  evening: /^z?ve/i,
                  night: /(po)?no/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sl/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'sl',
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
