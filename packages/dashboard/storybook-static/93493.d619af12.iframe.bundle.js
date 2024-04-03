(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [93493, 45585, 69107, 95565, 79489, 62127, 10906, 41504, 18048, 35624],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'mens d’una segonda', other: 'mens de {{count}} segondas' },
            xSeconds: { one: '1 segonda', other: '{{count}} segondas' },
            halfAMinute: '30 segondas',
            lessThanXMinutes: { one: 'mens d’una minuta', other: 'mens de {{count}} minutas' },
            xMinutes: { one: '1 minuta', other: '{{count}} minutas' },
            aboutXHours: { one: 'environ 1 ora', other: 'environ {{count}} oras' },
            xHours: { one: '1 ora', other: '{{count}} oras' },
            xDays: { one: '1 jorn', other: '{{count}} jorns' },
            aboutXWeeks: { one: 'environ 1 setmana', other: 'environ {{count}} setmanas' },
            xWeeks: { one: '1 setmana', other: '{{count}} setmanas' },
            aboutXMonths: { one: 'environ 1 mes', other: 'environ {{count}} meses' },
            xMonths: { one: '1 mes', other: '{{count}} meses' },
            aboutXYears: { one: 'environ 1 an', other: 'environ {{count}} ans' },
            xYears: { one: '1 an', other: '{{count}} ans' },
            overXYears: { one: 'mai d’un an', other: 'mai de {{count}} ans' },
            almostXYears: { one: 'gaireben un an', other: 'gaireben {{count}} ans' },
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
                  ? 'd’aquí ' + result
                  : 'fa ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatLong/index.js':
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
                full: "EEEE d 'de' MMMM y",
                long: "d 'de' MMMM y",
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
                full: "{{date}} 'a' {{time}}",
                long: "{{date}} 'a' {{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'passat a' p",
            yesterday: "'ièr a' p",
            today: "'uèi a' p",
            tomorrow: "'deman a' p",
            nextWeek: "eeee 'a' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/localize/index.js':
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
              var ordinal,
                number = Number(dirtyNumber),
                unit = null == options ? void 0 : options.unit;
              switch (number) {
                case 1:
                  ordinal = 'èr';
                  break;
                case 2:
                  ordinal = 'nd';
                  break;
                default:
                  ordinal = 'en';
              }
              return (
                ('year' !== unit &&
                  'week' !== unit &&
                  'hour' !== unit &&
                  'minute' !== unit &&
                  'second' !== unit) ||
                  (ordinal += 'a'),
                number + ordinal
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['ab. J.C.', 'apr. J.C.'],
                abbreviated: ['ab. J.C.', 'apr. J.C.'],
                wide: ['abans Jèsus-Crist', 'après Jèsus-Crist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['T1', 'T2', 'T3', 'T4'],
                abbreviated: ['1èr trim.', '2nd trim.', '3en trim.', '4en trim.'],
                wide: ['1èr trimèstre', '2nd trimèstre', '3en trimèstre', '4en trimèstre'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['GN', 'FB', 'MÇ', 'AB', 'MA', 'JN', 'JL', 'AG', 'ST', 'OC', 'NV', 'DC'],
                abbreviated: [
                  'gen.',
                  'febr.',
                  'març',
                  'abr.',
                  'mai',
                  'junh',
                  'jul.',
                  'ag.',
                  'set.',
                  'oct.',
                  'nov.',
                  'dec.',
                ],
                wide: [
                  'genièr',
                  'febrièr',
                  'març',
                  'abril',
                  'mai',
                  'junh',
                  'julhet',
                  'agost',
                  'setembre',
                  'octòbre',
                  'novembre',
                  'decembre',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['dg.', 'dl.', 'dm.', 'dc.', 'dj.', 'dv.', 'ds.'],
                short: ['dg.', 'dl.', 'dm.', 'dc.', 'dj.', 'dv.', 'ds.'],
                abbreviated: ['dg.', 'dl.', 'dm.', 'dc.', 'dj.', 'dv.', 'ds.'],
                wide: [
                  'dimenge',
                  'diluns',
                  'dimars',
                  'dimècres',
                  'dijòus',
                  'divendres',
                  'dissabte',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'am',
                  pm: 'pm',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'matin',
                  afternoon: 'aprèp-miègjorn',
                  evening: 'vèspre',
                  night: 'nuèch',
                },
                abbreviated: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'matin',
                  afternoon: 'aprèp-miègjorn',
                  evening: 'vèspre',
                  night: 'nuèch',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'matin',
                  afternoon: 'aprèp-miègjorn',
                  evening: 'vèspre',
                  night: 'nuèch',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'am',
                  pm: 'pm',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'del matin',
                  afternoon: 'de l’aprèp-miègjorn',
                  evening: 'del ser',
                  night: 'de la nuèch',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'del matin',
                  afternoon: 'de l’aprèp-miègjorn',
                  evening: 'del ser',
                  night: 'de la nuèch',
                },
                wide: {
                  am: 'ante meridiem',
                  pm: 'post meridiem',
                  midnight: 'mièjanuèch',
                  noon: 'miègjorn',
                  morning: 'del matin',
                  afternoon: 'de l’aprèp-miègjorn',
                  evening: 'del ser',
                  night: 'de la nuèch',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/match/index.js':
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
              matchPattern: /^(\d+)(èr|nd|en)?[a]?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ab\.J\.C|apr\.J\.C|apr\.J\.-C)/i,
                abbreviated: /^(ab\.J\.-C|ab\.J-C|apr\.J\.-C|apr\.J-C|ap\.J-C)/i,
                wide: /^(abans Jèsus-Crist|après Jèsus-Crist)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^ab/i, /^ap/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^T[1234]/i,
                abbreviated: /^[1234](èr|nd|en)? trim\.?/i,
                wide: /^[1234](èr|nd|en)? trimèstre/i,
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
                narrow: /^(GN|FB|MÇ|AB|MA|JN|JL|AG|ST|OC|NV|DC)/i,
                abbreviated: /^(gen|febr|març|abr|mai|junh|jul|ag|set|oct|nov|dec)\.?/i,
                wide: /^(genièr|febrièr|març|abril|mai|junh|julhet|agost|setembre|octòbre|novembre|decembre)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: [
                  /^g/i,
                  /^f/i,
                  /^ma[r?]|MÇ/i,
                  /^ab/i,
                  /^ma[i?]/i,
                  /^ju[n?]|JN/i,
                  /^ju[l?]|JL/i,
                  /^ag/i,
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
                narrow: /^d[glmcjvs]\.?/i,
                short: /^d[glmcjvs]\.?/i,
                abbreviated: /^d[glmcjvs]\.?/i,
                wide: /^(dimenge|diluns|dimars|dimècres|dijòus|divendres|dissabte)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^dg/i, /^dl/i, /^dm/i, /^dc/i, /^dj/i, /^dv/i, /^ds/i],
                short: [/^dg/i, /^dl/i, /^dm/i, /^dc/i, /^dj/i, /^dv/i, /^ds/i],
                abbreviated: [/^dg/i, /^dl/i, /^dm/i, /^dc/i, /^dj/i, /^dv/i, /^ds/i],
                any: [
                  /^dg|dime/i,
                  /^dl|dil/i,
                  /^dm|dima/i,
                  /^dc|dimè/i,
                  /^dj|dij/i,
                  /^dv|div/i,
                  /^ds|dis/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                any: /(^(a\.?m|p\.?m))|(ante meridiem|post meridiem)|((del |de la |de l’)(matin|aprèp-miègjorn|vèspre|ser|nuèch))/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /(^a)|ante meridiem/i,
                  pm: /(^p)|post meridiem/i,
                  midnight: /^mièj/i,
                  noon: /^mièg/i,
                  morning: /matin/i,
                  afternoon: /aprèp-miègjorn/i,
                  evening: /vèspre|ser/i,
                  night: /nuèch/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'oc',
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
