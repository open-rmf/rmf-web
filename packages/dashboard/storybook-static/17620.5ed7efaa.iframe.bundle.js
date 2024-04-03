(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [17620, 45585, 69107, 95565, 79489, 66898, 63683, 65457, 44281, 2167],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'секунд хүрэхгүй', other: '{{count}} секунд хүрэхгүй' },
            xSeconds: { one: '1 секунд', other: '{{count}} секунд' },
            halfAMinute: 'хагас минут',
            lessThanXMinutes: { one: 'минут хүрэхгүй', other: '{{count}} минут хүрэхгүй' },
            xMinutes: { one: '1 минут', other: '{{count}} минут' },
            aboutXHours: { one: 'ойролцоогоор 1 цаг', other: 'ойролцоогоор {{count}} цаг' },
            xHours: { one: '1 цаг', other: '{{count}} цаг' },
            xDays: { one: '1 өдөр', other: '{{count}} өдөр' },
            aboutXWeeks: {
              one: 'ойролцоогоор 1 долоо хоног',
              other: 'ойролцоогоор {{count}} долоо хоног',
            },
            xWeeks: { one: '1 долоо хоног', other: '{{count}} долоо хоног' },
            aboutXMonths: { one: 'ойролцоогоор 1 сар', other: 'ойролцоогоор {{count}} сар' },
            xMonths: { one: '1 сар', other: '{{count}} сар' },
            aboutXYears: { one: 'ойролцоогоор 1 жил', other: 'ойролцоогоор {{count}} жил' },
            xYears: { one: '1 жил', other: '{{count}} жил' },
            overXYears: { one: '1 жил гаран', other: '{{count}} жил гаран' },
            almostXYears: { one: 'бараг 1 жил', other: 'бараг {{count}} жил' },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue = formatDistanceLocale[token];
            if (
              ((result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix)
            ) {
              var words = result.split(' '),
                lastword = words.pop();
              switch (((result = words.join(' ')), lastword)) {
                case 'секунд':
                  result += ' секундийн';
                  break;
                case 'минут':
                  result += ' минутын';
                  break;
                case 'цаг':
                  result += ' цагийн';
                  break;
                case 'өдөр':
                  result += ' өдрийн';
                  break;
                case 'сар':
                  result += ' сарын';
                  break;
                case 'жил':
                  result += ' жилийн';
                  break;
                case 'хоног':
                  result += ' хоногийн';
                  break;
                case 'гаран':
                  result += ' гараны';
                  break;
                case 'хүрэхгүй':
                  result += ' хүрэхгүй хугацааны';
                  break;
                default:
                  result += lastword + '-н';
              }
              return options.comparison && options.comparison > 0
                ? result + ' дараа'
                : result + ' өмнө';
            }
            return result;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatLong/index.js':
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
                full: "y 'оны' MMMM'ын' d, EEEE 'гараг'",
                long: "y 'оны' MMMM'ын' d",
                medium: "y 'оны' MMM'ын' d",
                short: 'y.MM.dd',
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'өнгөрсөн' eeee 'гарагийн' p 'цагт'",
            yesterday: "'өчигдөр' p 'цагт'",
            today: "'өнөөдөр' p 'цагт'",
            tomorrow: "'маргааш' p 'цагт'",
            nextWeek: "'ирэх' eeee 'гарагийн' p 'цагт'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/localize/index.js':
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
                narrow: ['НТӨ', 'НТ'],
                abbreviated: ['НТӨ', 'НТ'],
                wide: ['нийтийн тооллын өмнөх', 'нийтийн тооллын'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['I', 'II', 'III', 'IV'],
                abbreviated: ['I улирал', 'II улирал', 'III улирал', 'IV улирал'],
                wide: ['1-р улирал', '2-р улирал', '3-р улирал', '4-р улирал'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['I', 'II', 'III', 'IV', 'V', 'VI', 'VII', 'VIII', 'IX', 'X', 'XI', 'XII'],
                abbreviated: [
                  '1-р сар',
                  '2-р сар',
                  '3-р сар',
                  '4-р сар',
                  '5-р сар',
                  '6-р сар',
                  '7-р сар',
                  '8-р сар',
                  '9-р сар',
                  '10-р сар',
                  '11-р сар',
                  '12-р сар',
                ],
                wide: [
                  'Нэгдүгээр сар',
                  'Хоёрдугаар сар',
                  'Гуравдугаар сар',
                  'Дөрөвдүгээр сар',
                  'Тавдугаар сар',
                  'Зургаадугаар сар',
                  'Долоодугаар сар',
                  'Наймдугаар сар',
                  'Есдүгээр сар',
                  'Аравдугаар сар',
                  'Арваннэгдүгээр сар',
                  'Арван хоёрдугаар сар',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['I', 'II', 'III', 'IV', 'V', 'VI', 'VII', 'VIII', 'IX', 'X', 'XI', 'XII'],
                abbreviated: [
                  '1-р сар',
                  '2-р сар',
                  '3-р сар',
                  '4-р сар',
                  '5-р сар',
                  '6-р сар',
                  '7-р сар',
                  '8-р сар',
                  '9-р сар',
                  '10-р сар',
                  '11-р сар',
                  '12-р сар',
                ],
                wide: [
                  'нэгдүгээр сар',
                  'хоёрдугаар сар',
                  'гуравдугаар сар',
                  'дөрөвдүгээр сар',
                  'тавдугаар сар',
                  'зургаадугаар сар',
                  'долоодугаар сар',
                  'наймдугаар сар',
                  'есдүгээр сар',
                  'аравдугаар сар',
                  'арваннэгдүгээр сар',
                  'арван хоёрдугаар сар',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Н', 'Д', 'М', 'Л', 'П', 'Б', 'Б'],
                short: ['Ня', 'Да', 'Мя', 'Лх', 'Пү', 'Ба', 'Бя'],
                abbreviated: ['Ням', 'Дав', 'Мяг', 'Лха', 'Пүр', 'Баа', 'Бям'],
                wide: ['Ням', 'Даваа', 'Мягмар', 'Лхагва', 'Пүрэв', 'Баасан', 'Бямба'],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['Н', 'Д', 'М', 'Л', 'П', 'Б', 'Б'],
                short: ['Ня', 'Да', 'Мя', 'Лх', 'Пү', 'Ба', 'Бя'],
                abbreviated: ['Ням', 'Дав', 'Мяг', 'Лха', 'Пүр', 'Баа', 'Бям'],
                wide: ['ням', 'даваа', 'мягмар', 'лхагва', 'пүрэв', 'баасан', 'бямба'],
              },
              defaultFormattingWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ү.ө.',
                  pm: 'ү.х.',
                  midnight: 'шөнө дунд',
                  noon: 'үд дунд',
                  morning: 'өглөө',
                  afternoon: 'өдөр',
                  evening: 'орой',
                  night: 'шөнө',
                },
                abbreviated: {
                  am: 'ү.ө.',
                  pm: 'ү.х.',
                  midnight: 'шөнө дунд',
                  noon: 'үд дунд',
                  morning: 'өглөө',
                  afternoon: 'өдөр',
                  evening: 'орой',
                  night: 'шөнө',
                },
                wide: {
                  am: 'ү.ө.',
                  pm: 'ү.х.',
                  midnight: 'шөнө дунд',
                  noon: 'үд дунд',
                  morning: 'өглөө',
                  afternoon: 'өдөр',
                  evening: 'орой',
                  night: 'шөнө',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/match/index.js':
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
              matchPattern: /\d+/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(нтө|нт)/i,
                abbreviated: /^(нтө|нт)/i,
                wide: /^(нийтийн тооллын өмнө|нийтийн тооллын)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^(нтө|нийтийн тооллын өмнө)/i, /^(нт|нийтийн тооллын)/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^(iv|iii|ii|i)/i,
                abbreviated: /^(iv|iii|ii|i) улирал/i,
                wide: /^[1-4]-р улирал/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: [/^(i(\s|$)|1)/i, /^(ii(\s|$)|2)/i, /^(iii(\s|$)|3)/i, /^(iv(\s|$)|4)/i],
              },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^(xii|xi|x|ix|viii|vii|vi|v|iv|iii|ii|i)/i,
                abbreviated:
                  /^(1-р сар|2-р сар|3-р сар|4-р сар|5-р сар|6-р сар|7-р сар|8-р сар|9-р сар|10-р сар|11-р сар|12-р сар)/i,
                wide: /^(нэгдүгээр сар|хоёрдугаар сар|гуравдугаар сар|дөрөвдүгээр сар|тавдугаар сар|зургаадугаар сар|долоодугаар сар|наймдугаар сар|есдүгээр сар|аравдугаар сар|арван нэгдүгээр сар|арван хоёрдугаар сар)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^i$/i,
                  /^ii$/i,
                  /^iii$/i,
                  /^iv$/i,
                  /^v$/i,
                  /^vi$/i,
                  /^vii$/i,
                  /^viii$/i,
                  /^ix$/i,
                  /^x$/i,
                  /^xi$/i,
                  /^xii$/i,
                ],
                any: [
                  /^(1|нэгдүгээр)/i,
                  /^(2|хоёрдугаар)/i,
                  /^(3|гуравдугаар)/i,
                  /^(4|дөрөвдүгээр)/i,
                  /^(5|тавдугаар)/i,
                  /^(6|зургаадугаар)/i,
                  /^(7|долоодугаар)/i,
                  /^(8|наймдугаар)/i,
                  /^(9|есдүгээр)/i,
                  /^(10|аравдугаар)/i,
                  /^(11|арван нэгдүгээр)/i,
                  /^(12|арван хоёрдугаар)/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[ндмлпбб]/i,
                short: /^(ня|да|мя|лх|пү|ба|бя)/i,
                abbreviated: /^(ням|дав|мяг|лха|пүр|баа|бям)/i,
                wide: /^(ням|даваа|мягмар|лхагва|пүрэв|баасан|бямба)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^н/i, /^д/i, /^м/i, /^л/i, /^п/i, /^б/i, /^б/i],
                any: [/^ня/i, /^да/i, /^мя/i, /^лх/i, /^пү/i, /^ба/i, /^бя/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ү\.ө\.|ү\.х\.|шөнө дунд|үд дунд|өглөө|өдөр|орой|шөнө)/i,
                any: /^(ү\.ө\.|ү\.х\.|шөнө дунд|үд дунд|өглөө|өдөр|орой|шөнө)/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^ү\.ө\./i,
                  pm: /^ү\.х\./i,
                  midnight: /^шөнө дунд/i,
                  noon: /^үд дунд/i,
                  morning: /өглөө/i,
                  afternoon: /өдөр/i,
                  evening: /орой/i,
                  night: /шөнө/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'mn',
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
