(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [71740, 45585, 69107, 95565, 79489, 95338, 73579, 43593, 50049, 5887],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var translations = {
            about: 'körülbelül',
            over: 'több mint',
            almost: 'majdnem',
            lessthan: 'kevesebb mint',
          },
          withoutSuffixes = {
            xseconds: ' másodperc',
            halfaminute: 'fél perc',
            xminutes: ' perc',
            xhours: ' óra',
            xdays: ' nap',
            xweeks: ' hét',
            xmonths: ' hónap',
            xyears: ' év',
          },
          withSuffixes = {
            xseconds: { '-1': ' másodperccel ezelőtt', 1: ' másodperc múlva', 0: ' másodperce' },
            halfaminute: { '-1': 'fél perccel ezelőtt', 1: 'fél perc múlva', 0: 'fél perce' },
            xminutes: { '-1': ' perccel ezelőtt', 1: ' perc múlva', 0: ' perce' },
            xhours: { '-1': ' órával ezelőtt', 1: ' óra múlva', 0: ' órája' },
            xdays: { '-1': ' nappal ezelőtt', 1: ' nap múlva', 0: ' napja' },
            xweeks: { '-1': ' héttel ezelőtt', 1: ' hét múlva', 0: ' hete' },
            xmonths: { '-1': ' hónappal ezelőtt', 1: ' hónap múlva', 0: ' hónapja' },
            xyears: { '-1': ' évvel ezelőtt', 1: ' év múlva', 0: ' éve' },
          },
          _default = function formatDistance(token, count, options) {
            var adverb = token.match(/about|over|almost|lessthan/i),
              unit = adverb ? token.replace(adverb[0], '') : token,
              addSuffix = !0 === (null == options ? void 0 : options.addSuffix),
              key = unit.toLowerCase(),
              comparison = (null == options ? void 0 : options.comparison) || 0,
              translated = addSuffix ? withSuffixes[key][comparison] : withoutSuffixes[key],
              result = 'halfaminute' === key ? translated : count + translated;
            if (adverb) {
              var adv = adverb[0].toLowerCase();
              result = translations[adv] + ' ' + result;
            }
            return result;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatLong/index.js':
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
                full: 'y. MMMM d., EEEE',
                long: 'y. MMMM d.',
                medium: 'y. MMM d.',
                short: 'y. MM. dd.',
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var accusativeWeekdays = [
          'vasárnap',
          'hétfőn',
          'kedden',
          'szerdán',
          'csütörtökön',
          'pénteken',
          'szombaton',
        ];
        function week(isFuture) {
          return function (date) {
            var weekday = accusativeWeekdays[date.getUTCDay()];
            return ''.concat(isFuture ? '' : "'múlt' ", "'").concat(weekday, "' p'-kor'");
          };
        }
        var formatRelativeLocale = {
            lastWeek: week(!1),
            yesterday: "'tegnap' p'-kor'",
            today: "'ma' p'-kor'",
            tomorrow: "'holnap' p'-kor'",
            nextWeek: week(!0),
            other: 'P',
          },
          _default = function formatRelative(token, date) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/localize/index.js':
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
                narrow: ['ie.', 'isz.'],
                abbreviated: ['i. e.', 'i. sz.'],
                wide: ['Krisztus előtt', 'időszámításunk szerint'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1.', '2.', '3.', '4.'],
                abbreviated: ['1. n.év', '2. n.év', '3. n.év', '4. n.év'],
                wide: ['1. negyedév', '2. negyedév', '3. negyedév', '4. negyedév'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
              formattingValues: {
                narrow: ['I.', 'II.', 'III.', 'IV.'],
                abbreviated: ['I. n.év', 'II. n.év', 'III. n.év', 'IV. n.év'],
                wide: ['I. negyedév', 'II. negyedév', 'III. negyedév', 'IV. negyedév'],
              },
              defaultFormattingWidth: 'wide',
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['J', 'F', 'M', 'Á', 'M', 'J', 'J', 'A', 'Sz', 'O', 'N', 'D'],
                abbreviated: [
                  'jan.',
                  'febr.',
                  'márc.',
                  'ápr.',
                  'máj.',
                  'jún.',
                  'júl.',
                  'aug.',
                  'szept.',
                  'okt.',
                  'nov.',
                  'dec.',
                ],
                wide: [
                  'január',
                  'február',
                  'március',
                  'április',
                  'május',
                  'június',
                  'július',
                  'augusztus',
                  'szeptember',
                  'október',
                  'november',
                  'december',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['V', 'H', 'K', 'Sz', 'Cs', 'P', 'Sz'],
                short: ['V', 'H', 'K', 'Sze', 'Cs', 'P', 'Szo'],
                abbreviated: ['V', 'H', 'K', 'Sze', 'Cs', 'P', 'Szo'],
                wide: ['vasárnap', 'hétfő', 'kedd', 'szerda', 'csütörtök', 'péntek', 'szombat'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'de.',
                  pm: 'du.',
                  midnight: 'éjfél',
                  noon: 'dél',
                  morning: 'reggel',
                  afternoon: 'du.',
                  evening: 'este',
                  night: 'éjjel',
                },
                abbreviated: {
                  am: 'de.',
                  pm: 'du.',
                  midnight: 'éjfél',
                  noon: 'dél',
                  morning: 'reggel',
                  afternoon: 'du.',
                  evening: 'este',
                  night: 'éjjel',
                },
                wide: {
                  am: 'de.',
                  pm: 'du.',
                  midnight: 'éjfél',
                  noon: 'dél',
                  morning: 'reggel',
                  afternoon: 'délután',
                  evening: 'este',
                  night: 'éjjel',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/match/index.js':
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
              matchPattern: /^(\d+)\.?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ie\.|isz\.)/i,
                abbreviated: /^(i\.\s?e\.?|b?\s?c\s?e|i\.\s?sz\.?)/i,
                wide: /^(Krisztus előtt|időszámításunk előtt|időszámításunk szerint|i\. sz\.)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/ie/i, /isz/i],
                abbreviated: [/^(i\.?\s?e\.?|b\s?ce)/i, /^(i\.?\s?sz\.?|c\s?e)/i],
                any: [/előtt/i, /(szerint|i. sz.)/i],
              },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]\.?/i,
                abbreviated: /^[1234]?\.?\s?n\.év/i,
                wide: /^([1234]|I|II|III|IV)?\.?\s?negyedév/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/1|I$/i, /2|II$/i, /3|III/i, /4|IV/i] },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^[jfmaásond]|sz/i,
                abbreviated:
                  /^(jan\.?|febr\.?|márc\.?|ápr\.?|máj\.?|jún\.?|júl\.?|aug\.?|szept\.?|okt\.?|nov\.?|dec\.?)/i,
                wide: /^(január|február|március|április|május|június|július|augusztus|szeptember|október|november|december)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^j/i,
                  /^f/i,
                  /^m/i,
                  /^a|á/i,
                  /^m/i,
                  /^j/i,
                  /^j/i,
                  /^a/i,
                  /^s|sz/i,
                  /^o/i,
                  /^n/i,
                  /^d/i,
                ],
                any: [
                  /^ja/i,
                  /^f/i,
                  /^már/i,
                  /^áp/i,
                  /^máj/i,
                  /^jún/i,
                  /^júl/i,
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
                narrow: /^([vhkpc]|sz|cs|sz)/i,
                short: /^([vhkp]|sze|cs|szo)/i,
                abbreviated: /^([vhkp]|sze|cs|szo)/i,
                wide: /^(vasárnap|hétfő|kedd|szerda|csütörtök|péntek|szombat)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^v/i, /^h/i, /^k/i, /^sz/i, /^c/i, /^p/i, /^sz/i],
                any: [/^v/i, /^h/i, /^k/i, /^sze/i, /^c/i, /^p/i, /^szo/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: { any: /^((de|du)\.?|éjfél|délután|dél|reggel|este|éjjel)/i },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^de\.?/i,
                  pm: /^du\.?/i,
                  midnight: /^éjf/i,
                  noon: /^dé/i,
                  morning: /reg/i,
                  afternoon: /^délu\.?/i,
                  evening: /es/i,
                  night: /éjj/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'hu',
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
