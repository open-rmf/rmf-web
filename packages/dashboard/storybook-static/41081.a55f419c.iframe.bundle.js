(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [41081, 45585, 69107, 95565, 79489, 26867, 95614, 65004, 31316, 26228],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: {
                regular: 'méně než sekunda',
                past: 'před méně než sekundou',
                future: 'za méně než sekundu',
              },
              few: {
                regular: 'méně než {{count}} sekundy',
                past: 'před méně než {{count}} sekundami',
                future: 'za méně než {{count}} sekundy',
              },
              many: {
                regular: 'méně než {{count}} sekund',
                past: 'před méně než {{count}} sekundami',
                future: 'za méně než {{count}} sekund',
              },
            },
            xSeconds: {
              one: { regular: 'sekunda', past: 'před sekundou', future: 'za sekundu' },
              few: {
                regular: '{{count}} sekundy',
                past: 'před {{count}} sekundami',
                future: 'za {{count}} sekundy',
              },
              many: {
                regular: '{{count}} sekund',
                past: 'před {{count}} sekundami',
                future: 'za {{count}} sekund',
              },
            },
            halfAMinute: {
              type: 'other',
              other: { regular: 'půl minuty', past: 'před půl minutou', future: 'za půl minuty' },
            },
            lessThanXMinutes: {
              one: {
                regular: 'méně než minuta',
                past: 'před méně než minutou',
                future: 'za méně než minutu',
              },
              few: {
                regular: 'méně než {{count}} minuty',
                past: 'před méně než {{count}} minutami',
                future: 'za méně než {{count}} minuty',
              },
              many: {
                regular: 'méně než {{count}} minut',
                past: 'před méně než {{count}} minutami',
                future: 'za méně než {{count}} minut',
              },
            },
            xMinutes: {
              one: { regular: 'minuta', past: 'před minutou', future: 'za minutu' },
              few: {
                regular: '{{count}} minuty',
                past: 'před {{count}} minutami',
                future: 'za {{count}} minuty',
              },
              many: {
                regular: '{{count}} minut',
                past: 'před {{count}} minutami',
                future: 'za {{count}} minut',
              },
            },
            aboutXHours: {
              one: {
                regular: 'přibližně hodina',
                past: 'přibližně před hodinou',
                future: 'přibližně za hodinu',
              },
              few: {
                regular: 'přibližně {{count}} hodiny',
                past: 'přibližně před {{count}} hodinami',
                future: 'přibližně za {{count}} hodiny',
              },
              many: {
                regular: 'přibližně {{count}} hodin',
                past: 'přibližně před {{count}} hodinami',
                future: 'přibližně za {{count}} hodin',
              },
            },
            xHours: {
              one: { regular: 'hodina', past: 'před hodinou', future: 'za hodinu' },
              few: {
                regular: '{{count}} hodiny',
                past: 'před {{count}} hodinami',
                future: 'za {{count}} hodiny',
              },
              many: {
                regular: '{{count}} hodin',
                past: 'před {{count}} hodinami',
                future: 'za {{count}} hodin',
              },
            },
            xDays: {
              one: { regular: 'den', past: 'před dnem', future: 'za den' },
              few: {
                regular: '{{count}} dny',
                past: 'před {{count}} dny',
                future: 'za {{count}} dny',
              },
              many: {
                regular: '{{count}} dní',
                past: 'před {{count}} dny',
                future: 'za {{count}} dní',
              },
            },
            aboutXWeeks: {
              one: {
                regular: 'přibližně týden',
                past: 'přibližně před týdnem',
                future: 'přibližně za týden',
              },
              few: {
                regular: 'přibližně {{count}} týdny',
                past: 'přibližně před {{count}} týdny',
                future: 'přibližně za {{count}} týdny',
              },
              many: {
                regular: 'přibližně {{count}} týdnů',
                past: 'přibližně před {{count}} týdny',
                future: 'přibližně za {{count}} týdnů',
              },
            },
            xWeeks: {
              one: { regular: 'týden', past: 'před týdnem', future: 'za týden' },
              few: {
                regular: '{{count}} týdny',
                past: 'před {{count}} týdny',
                future: 'za {{count}} týdny',
              },
              many: {
                regular: '{{count}} týdnů',
                past: 'před {{count}} týdny',
                future: 'za {{count}} týdnů',
              },
            },
            aboutXMonths: {
              one: {
                regular: 'přibližně měsíc',
                past: 'přibližně před měsícem',
                future: 'přibližně za měsíc',
              },
              few: {
                regular: 'přibližně {{count}} měsíce',
                past: 'přibližně před {{count}} měsíci',
                future: 'přibližně za {{count}} měsíce',
              },
              many: {
                regular: 'přibližně {{count}} měsíců',
                past: 'přibližně před {{count}} měsíci',
                future: 'přibližně za {{count}} měsíců',
              },
            },
            xMonths: {
              one: { regular: 'měsíc', past: 'před měsícem', future: 'za měsíc' },
              few: {
                regular: '{{count}} měsíce',
                past: 'před {{count}} měsíci',
                future: 'za {{count}} měsíce',
              },
              many: {
                regular: '{{count}} měsíců',
                past: 'před {{count}} měsíci',
                future: 'za {{count}} měsíců',
              },
            },
            aboutXYears: {
              one: {
                regular: 'přibližně rok',
                past: 'přibližně před rokem',
                future: 'přibližně za rok',
              },
              few: {
                regular: 'přibližně {{count}} roky',
                past: 'přibližně před {{count}} roky',
                future: 'přibližně za {{count}} roky',
              },
              many: {
                regular: 'přibližně {{count}} roků',
                past: 'přibližně před {{count}} roky',
                future: 'přibližně za {{count}} roků',
              },
            },
            xYears: {
              one: { regular: 'rok', past: 'před rokem', future: 'za rok' },
              few: {
                regular: '{{count}} roky',
                past: 'před {{count}} roky',
                future: 'za {{count}} roky',
              },
              many: {
                regular: '{{count}} roků',
                past: 'před {{count}} roky',
                future: 'za {{count}} roků',
              },
            },
            overXYears: {
              one: {
                regular: 'více než rok',
                past: 'před více než rokem',
                future: 'za více než rok',
              },
              few: {
                regular: 'více než {{count}} roky',
                past: 'před více než {{count}} roky',
                future: 'za více než {{count}} roky',
              },
              many: {
                regular: 'více než {{count}} roků',
                past: 'před více než {{count}} roky',
                future: 'za více než {{count}} roků',
              },
            },
            almostXYears: {
              one: { regular: 'skoro rok', past: 'skoro před rokem', future: 'skoro za rok' },
              few: {
                regular: 'skoro {{count}} roky',
                past: 'skoro před {{count}} roky',
                future: 'skoro za {{count}} roky',
              },
              many: {
                regular: 'skoro {{count}} roků',
                past: 'skoro před {{count}} roky',
                future: 'skoro za {{count}} roků',
              },
            },
          },
          _default = function formatDistance(token, count, options) {
            var pluralResult,
              tokenValue = formatDistanceLocale[token];
            pluralResult =
              'other' === tokenValue.type
                ? tokenValue.other
                : 1 === count
                  ? tokenValue.one
                  : count > 1 && count < 5
                    ? tokenValue.few
                    : tokenValue.many;
            var suffixExist = !0 === (null == options ? void 0 : options.addSuffix),
              comparison = null == options ? void 0 : options.comparison;
            return (
              suffixExist && -1 === comparison
                ? pluralResult.past
                : suffixExist && 1 === comparison
                  ? pluralResult.future
                  : pluralResult.regular
            ).replace('{{count}}', String(count));
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatLong/index.js':
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
                full: 'EEEE, d. MMMM yyyy',
                long: 'd. MMMM yyyy',
                medium: 'd. M. yyyy',
                short: 'dd.MM.yyyy',
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
                full: "{{date}} 'v' {{time}}",
                long: "{{date}} 'v' {{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var accusativeWeekdays = [
            'neděli',
            'pondělí',
            'úterý',
            'středu',
            'čtvrtek',
            'pátek',
            'sobotu',
          ],
          formatRelativeLocale = {
            lastWeek: "'poslední' eeee 've' p",
            yesterday: "'včera v' p",
            today: "'dnes v' p",
            tomorrow: "'zítra v' p",
            nextWeek: function nextWeek(date) {
              var day = date.getUTCDay();
              return "'v " + accusativeWeekdays[day] + " o' p";
            },
            other: 'P',
          },
          _default = function formatRelative(token, date) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/localize/index.js':
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
                narrow: ['př. n. l.', 'n. l.'],
                abbreviated: ['př. n. l.', 'n. l.'],
                wide: ['před naším letopočtem', 'našeho letopočtu'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1. čtvrtletí', '2. čtvrtletí', '3. čtvrtletí', '4. čtvrtletí'],
                wide: ['1. čtvrtletí', '2. čtvrtletí', '3. čtvrtletí', '4. čtvrtletí'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['L', 'Ú', 'B', 'D', 'K', 'Č', 'Č', 'S', 'Z', 'Ř', 'L', 'P'],
                abbreviated: [
                  'led',
                  'úno',
                  'bře',
                  'dub',
                  'kvě',
                  'čvn',
                  'čvc',
                  'srp',
                  'zář',
                  'říj',
                  'lis',
                  'pro',
                ],
                wide: [
                  'leden',
                  'únor',
                  'březen',
                  'duben',
                  'květen',
                  'červen',
                  'červenec',
                  'srpen',
                  'září',
                  'říjen',
                  'listopad',
                  'prosinec',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['L', 'Ú', 'B', 'D', 'K', 'Č', 'Č', 'S', 'Z', 'Ř', 'L', 'P'],
                abbreviated: [
                  'led',
                  'úno',
                  'bře',
                  'dub',
                  'kvě',
                  'čvn',
                  'čvc',
                  'srp',
                  'zář',
                  'říj',
                  'lis',
                  'pro',
                ],
                wide: [
                  'ledna',
                  'února',
                  'března',
                  'dubna',
                  'května',
                  'června',
                  'července',
                  'srpna',
                  'září',
                  'října',
                  'listopadu',
                  'prosince',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ne', 'po', 'út', 'st', 'čt', 'pá', 'so'],
                short: ['ne', 'po', 'út', 'st', 'čt', 'pá', 'so'],
                abbreviated: ['ned', 'pon', 'úte', 'stř', 'čtv', 'pát', 'sob'],
                wide: ['neděle', 'pondělí', 'úterý', 'středa', 'čtvrtek', 'pátek', 'sobota'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'dop.',
                  pm: 'odp.',
                  midnight: 'půlnoc',
                  noon: 'poledne',
                  morning: 'ráno',
                  afternoon: 'odpoledne',
                  evening: 'večer',
                  night: 'noc',
                },
                abbreviated: {
                  am: 'dop.',
                  pm: 'odp.',
                  midnight: 'půlnoc',
                  noon: 'poledne',
                  morning: 'ráno',
                  afternoon: 'odpoledne',
                  evening: 'večer',
                  night: 'noc',
                },
                wide: {
                  am: 'dopoledne',
                  pm: 'odpoledne',
                  midnight: 'půlnoc',
                  noon: 'poledne',
                  morning: 'ráno',
                  afternoon: 'odpoledne',
                  evening: 'večer',
                  night: 'noc',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'dop.',
                  pm: 'odp.',
                  midnight: 'půlnoc',
                  noon: 'poledne',
                  morning: 'ráno',
                  afternoon: 'odpoledne',
                  evening: 'večer',
                  night: 'noc',
                },
                abbreviated: {
                  am: 'dop.',
                  pm: 'odp.',
                  midnight: 'půlnoc',
                  noon: 'poledne',
                  morning: 'ráno',
                  afternoon: 'odpoledne',
                  evening: 'večer',
                  night: 'noc',
                },
                wide: {
                  am: 'dopoledne',
                  pm: 'odpoledne',
                  midnight: 'půlnoc',
                  noon: 'poledne',
                  morning: 'ráno',
                  afternoon: 'odpoledne',
                  evening: 'večer',
                  night: 'noc',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/match/index.js':
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
                narrow: /^(p[řr](\.|ed) Kr\.|p[řr](\.|ed) n\. l\.|po Kr\.|n\. l\.)/i,
                abbreviated: /^(p[řr](\.|ed) Kr\.|p[řr](\.|ed) n\. l\.|po Kr\.|n\. l\.)/i,
                wide: /^(p[řr](\.|ed) Kristem|p[řr](\.|ed) na[šs][íi]m letopo[čc]tem|po Kristu|na[šs]eho letopo[čc]tu)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^p[řr]/i, /^(po|n)/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234]\. [čc]tvrtlet[íi]/i,
                wide: /^[1234]\. [čc]tvrtlet[íi]/i,
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
                narrow: /^[lúubdkčcszřrlp]/i,
                abbreviated:
                  /^(led|[úu]no|b[řr]e|dub|kv[ěe]|[čc]vn|[čc]vc|srp|z[áa][řr]|[řr][íi]j|lis|pro)/i,
                wide: /^(leden|ledna|[úu]nora?|b[řr]ezen|b[řr]ezna|duben|dubna|kv[ěe]ten|kv[ěe]tna|[čc]erven(ec|ce)?|[čc]ervna|srpen|srpna|z[áa][řr][íi]|[řr][íi]jen|[řr][íi]jna|listopad(a|u)?|prosinec|prosince)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^l/i,
                  /^[úu]/i,
                  /^b/i,
                  /^d/i,
                  /^k/i,
                  /^[čc]/i,
                  /^[čc]/i,
                  /^s/i,
                  /^z/i,
                  /^[řr]/i,
                  /^l/i,
                  /^p/i,
                ],
                any: [
                  /^led/i,
                  /^[úu]n/i,
                  /^b[řr]e/i,
                  /^dub/i,
                  /^kv[ěe]/i,
                  /^[čc]vn|[čc]erven(?!\w)|[čc]ervna/i,
                  /^[čc]vc|[čc]erven(ec|ce)/i,
                  /^srp/i,
                  /^z[áa][řr]/i,
                  /^[řr][íi]j/i,
                  /^lis/i,
                  /^pro/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[npuúsčps]/i,
                short: /^(ne|po|[úu]t|st|[čc]t|p[áa]|so)/i,
                abbreviated: /^(ned|pon|[úu]te|st[rř]|[čc]tv|p[áa]t|sob)/i,
                wide: /^(ned[ěe]le|pond[ěe]l[íi]|[úu]ter[ýy]|st[řr]eda|[čc]tvrtek|p[áa]tek|sobota)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^n/i, /^p/i, /^[úu]/i, /^s/i, /^[čc]/i, /^p/i, /^s/i],
                any: [/^ne/i, /^po/i, /^[úu]t/i, /^st/i, /^[čc]t/i, /^p[áa]/i, /^so/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                any: /^dopoledne|dop\.?|odpoledne|odp\.?|p[ůu]lnoc|poledne|r[áa]no|odpoledne|ve[čc]er|(v )?noci?/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^dop/i,
                  pm: /^odp/i,
                  midnight: /^p[ůu]lnoc/i,
                  noon: /^poledne/i,
                  morning: /r[áa]no/i,
                  afternoon: /odpoledne/i,
                  evening: /ve[čc]er/i,
                  night: /noc/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'cs',
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
