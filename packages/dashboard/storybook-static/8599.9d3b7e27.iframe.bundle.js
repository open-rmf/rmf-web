(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [8599, 45585, 69107, 95565, 79489, 72537, 58164, 27934, 27294, 28154],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: "menys d'un segon",
              eleven: "menys d'onze segons",
              other: 'menys de {{count}} segons',
            },
            xSeconds: { one: '1 segon', other: '{{count}} segons' },
            halfAMinute: 'mig minut',
            lessThanXMinutes: {
              one: "menys d'un minut",
              eleven: "menys d'onze minuts",
              other: 'menys de {{count}} minuts',
            },
            xMinutes: { one: '1 minut', other: '{{count}} minuts' },
            aboutXHours: {
              one: 'aproximadament una hora',
              other: 'aproximadament {{count}} hores',
            },
            xHours: { one: '1 hora', other: '{{count}} hores' },
            xDays: { one: '1 dia', other: '{{count}} dies' },
            aboutXWeeks: {
              one: 'aproximadament una setmana',
              other: 'aproximadament {{count}} setmanes',
            },
            xWeeks: { one: '1 setmana', other: '{{count}} setmanes' },
            aboutXMonths: { one: 'aproximadament un mes', other: 'aproximadament {{count}} mesos' },
            xMonths: { one: '1 mes', other: '{{count}} mesos' },
            aboutXYears: { one: 'aproximadament un any', other: 'aproximadament {{count}} anys' },
            xYears: { one: '1 any', other: '{{count}} anys' },
            overXYears: {
              one: "més d'un any",
              eleven: "més d'onze anys",
              other: 'més de {{count}} anys',
            },
            almostXYears: { one: 'gairebé un any', other: 'gairebé {{count}} anys' },
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
                    : 11 === count && tokenValue.eleven
                      ? tokenValue.eleven
                      : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'en ' + result
                  : 'fa ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatLong/index.js':
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
                full: "EEEE, d 'de' MMMM y",
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
                full: "{{date}} 'a les' {{time}}",
                long: "{{date}} 'a les' {{time}}",
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'el' eeee 'passat a la' LT",
            yesterday: "'ahir a la' p",
            today: "'avui a la' p",
            tomorrow: "'demà a la' p",
            nextWeek: "eeee 'a la' p",
            other: 'P',
          },
          formatRelativeLocalePlural = {
            lastWeek: "'el' eeee 'passat a les' p",
            yesterday: "'ahir a les' p",
            today: "'avui a les' p",
            tomorrow: "'demà a les' p",
            nextWeek: "eeee 'a les' p",
            other: 'P',
          },
          _default = function formatRelative(token, date, _baseDate, _options) {
            return 1 !== date.getUTCHours()
              ? formatRelativeLocalePlural[token]
              : formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/localize/index.js':
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
              if (rem100 > 20 || rem100 < 10)
                switch (rem100 % 10) {
                  case 1:
                  case 3:
                    return number + 'r';
                  case 2:
                    return number + 'n';
                  case 4:
                    return number + 't';
                }
              return number + 'è';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['aC', 'dC'],
                abbreviated: ['a. de C.', 'd. de C.'],
                wide: ['abans de Crist', 'després de Crist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['T1', 'T2', 'T3', 'T4'],
                wide: ['1r trimestre', '2n trimestre', '3r trimestre', '4t trimestre'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['GN', 'FB', 'MÇ', 'AB', 'MG', 'JN', 'JL', 'AG', 'ST', 'OC', 'NV', 'DS'],
                abbreviated: [
                  'gen.',
                  'febr.',
                  'març',
                  'abr.',
                  'maig',
                  'juny',
                  'jul.',
                  'ag.',
                  'set.',
                  'oct.',
                  'nov.',
                  'des.',
                ],
                wide: [
                  'gener',
                  'febrer',
                  'març',
                  'abril',
                  'maig',
                  'juny',
                  'juliol',
                  'agost',
                  'setembre',
                  'octubre',
                  'novembre',
                  'desembre',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['dg.', 'dl.', 'dt.', 'dm.', 'dj.', 'dv.', 'ds.'],
                short: ['dg.', 'dl.', 'dt.', 'dm.', 'dj.', 'dv.', 'ds.'],
                abbreviated: ['dg.', 'dl.', 'dt.', 'dm.', 'dj.', 'dv.', 'ds.'],
                wide: [
                  'diumenge',
                  'dilluns',
                  'dimarts',
                  'dimecres',
                  'dijous',
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
                  midnight: 'mitjanit',
                  noon: 'migdia',
                  morning: 'matí',
                  afternoon: 'tarda',
                  evening: 'vespre',
                  night: 'nit',
                },
                abbreviated: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'mitjanit',
                  noon: 'migdia',
                  morning: 'matí',
                  afternoon: 'tarda',
                  evening: 'vespre',
                  night: 'nit',
                },
                wide: {
                  am: 'ante meridiem',
                  pm: 'post meridiem',
                  midnight: 'mitjanit',
                  noon: 'migdia',
                  morning: 'matí',
                  afternoon: 'tarda',
                  evening: 'vespre',
                  night: 'nit',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'am',
                  pm: 'pm',
                  midnight: 'de la mitjanit',
                  noon: 'del migdia',
                  morning: 'del matí',
                  afternoon: 'de la tarda',
                  evening: 'del vespre',
                  night: 'de la nit',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'de la mitjanit',
                  noon: 'del migdia',
                  morning: 'del matí',
                  afternoon: 'de la tarda',
                  evening: 'del vespre',
                  night: 'de la nit',
                },
                wide: {
                  am: 'ante meridiem',
                  pm: 'post meridiem',
                  midnight: 'de la mitjanit',
                  noon: 'del migdia',
                  morning: 'del matí',
                  afternoon: 'de la tarda',
                  evening: 'del vespre',
                  night: 'de la nit',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/match/index.js':
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
              matchPattern: /^(\d+)(è|r|n|r|t)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(aC|dC)/i,
                abbreviated: /^(a. de C.|d. de C.)/i,
                wide: /^(abans de Crist|despr[eé]s de Crist)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^aC/i, /^dC/i],
                abbreviated: [/^(a. de C.)/i, /^(d. de C.)/i],
                wide: [/^(abans de Crist)/i, /^(despr[eé]s de Crist)/i],
              },
              defaultParseWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^T[1234]/i,
                wide: /^[1234](è|r|n|r|t)? trimestre/i,
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
                narrow: /^(GN|FB|MÇ|AB|MG|JN|JL|AG|ST|OC|NV|DS)/i,
                abbreviated: /^(gen.|febr.|març|abr.|maig|juny|jul.|ag.|set.|oct.|nov.|des.)/i,
                wide: /^(gener|febrer|març|abril|maig|juny|juliol|agost|setembre|octubre|novembre|desembre)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^GN/i,
                  /^FB/i,
                  /^MÇ/i,
                  /^AB/i,
                  /^MG/i,
                  /^JN/i,
                  /^JL/i,
                  /^AG/i,
                  /^ST/i,
                  /^OC/i,
                  /^NV/i,
                  /^DS/i,
                ],
                abbreviated: [
                  /^gen./i,
                  /^febr./i,
                  /^març/i,
                  /^abr./i,
                  /^maig/i,
                  /^juny/i,
                  /^jul./i,
                  /^ag./i,
                  /^set./i,
                  /^oct./i,
                  /^nov./i,
                  /^des./i,
                ],
                wide: [
                  /^gener/i,
                  /^febrer/i,
                  /^març/i,
                  /^abril/i,
                  /^maig/i,
                  /^juny/i,
                  /^juliol/i,
                  /^agost/i,
                  /^setembre/i,
                  /^octubre/i,
                  /^novembre/i,
                  /^desembre/i,
                ],
              },
              defaultParseWidth: 'wide',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(dg\.|dl\.|dt\.|dm\.|dj\.|dv\.|ds\.)/i,
                short: /^(dg\.|dl\.|dt\.|dm\.|dj\.|dv\.|ds\.)/i,
                abbreviated: /^(dg\.|dl\.|dt\.|dm\.|dj\.|dv\.|ds\.)/i,
                wide: /^(diumenge|dilluns|dimarts|dimecres|dijous|divendres|dissabte)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^dg./i, /^dl./i, /^dt./i, /^dm./i, /^dj./i, /^dv./i, /^ds./i],
                abbreviated: [/^dg./i, /^dl./i, /^dt./i, /^dm./i, /^dj./i, /^dv./i, /^ds./i],
                wide: [
                  /^diumenge/i,
                  /^dilluns/i,
                  /^dimarts/i,
                  /^dimecres/i,
                  /^dijous/i,
                  /^divendres/i,
                  /^disssabte/i,
                ],
              },
              defaultParseWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(a|p|mn|md|(del|de la) (matí|tarda|vespre|nit))/i,
                abbreviated:
                  /^([ap]\.?\s?m\.?|mitjanit|migdia|(del|de la) (matí|tarda|vespre|nit))/i,
                wide: /^(ante meridiem|post meridiem|mitjanit|migdia|(del|de la) (matí|tarda|vespre|nit))/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: {
                  am: /^a/i,
                  pm: /^p/i,
                  midnight: /^mitjanit/i,
                  noon: /^migdia/i,
                  morning: /matí/i,
                  afternoon: /tarda/i,
                  evening: /vespre/i,
                  night: /nit/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'ca',
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
