(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [80245, 45585, 69107, 95565, 79489, 17231, 86074, 80736, 38144, 31976],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: '少於 1 秒', other: '少於 {{count}} 秒' },
            xSeconds: { one: '1 秒', other: '{{count}} 秒' },
            halfAMinute: '半分鐘',
            lessThanXMinutes: { one: '少於 1 分鐘', other: '少於 {{count}} 分鐘' },
            xMinutes: { one: '1 分鐘', other: '{{count}} 分鐘' },
            xHours: { one: '1 小時', other: '{{count}} 小時' },
            aboutXHours: { one: '大約 1 小時', other: '大約 {{count}} 小時' },
            xDays: { one: '1 天', other: '{{count}} 天' },
            aboutXWeeks: { one: '大約 1 個星期', other: '大約 {{count}} 個星期' },
            xWeeks: { one: '1 個星期', other: '{{count}} 個星期' },
            aboutXMonths: { one: '大約 1 個月', other: '大約 {{count}} 個月' },
            xMonths: { one: '1 個月', other: '{{count}} 個月' },
            aboutXYears: { one: '大約 1 年', other: '大約 {{count}} 年' },
            xYears: { one: '1 年', other: '{{count}} 年' },
            overXYears: { one: '超過 1 年', other: '超過 {{count}} 年' },
            almostXYears: { one: '將近 1 年', other: '將近 {{count}} 年' },
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
                  ? result + '內'
                  : result + '前'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatLong/index.js':
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
                full: "y'年'M'月'd'日' EEEE",
                long: "y'年'M'月'd'日'",
                medium: 'yyyy-MM-dd',
                short: 'yy-MM-dd',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'zzzz a h:mm:ss',
                long: 'z a h:mm:ss',
                medium: 'a h:mm:ss',
                short: 'a h:mm',
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'上個'eeee p",
            yesterday: "'昨天' p",
            today: "'今天' p",
            tomorrow: "'明天' p",
            nextWeek: "'下個'eeee p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/localize/index.js':
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
              var number = Number(dirtyNumber);
              switch (null == options ? void 0 : options.unit) {
                case 'date':
                  return number + '日';
                case 'hour':
                  return number + '時';
                case 'minute':
                  return number + '分';
                case 'second':
                  return number + '秒';
                default:
                  return '第 ' + number;
              }
            },
            era: (0, _index.default)({
              values: {
                narrow: ['前', '公元'],
                abbreviated: ['前', '公元'],
                wide: ['公元前', '公元'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['第一刻', '第二刻', '第三刻', '第四刻'],
                wide: ['第一刻鐘', '第二刻鐘', '第三刻鐘', '第四刻鐘'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: [
                  '一',
                  '二',
                  '三',
                  '四',
                  '五',
                  '六',
                  '七',
                  '八',
                  '九',
                  '十',
                  '十一',
                  '十二',
                ],
                abbreviated: [
                  '1月',
                  '2月',
                  '3月',
                  '4月',
                  '5月',
                  '6月',
                  '7月',
                  '8月',
                  '9月',
                  '10月',
                  '11月',
                  '12月',
                ],
                wide: [
                  '一月',
                  '二月',
                  '三月',
                  '四月',
                  '五月',
                  '六月',
                  '七月',
                  '八月',
                  '九月',
                  '十月',
                  '十一月',
                  '十二月',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['日', '一', '二', '三', '四', '五', '六'],
                short: ['日', '一', '二', '三', '四', '五', '六'],
                abbreviated: ['週日', '週一', '週二', '週三', '週四', '週五', '週六'],
                wide: ['星期日', '星期一', '星期二', '星期三', '星期四', '星期五', '星期六'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: '上',
                  pm: '下',
                  midnight: '凌晨',
                  noon: '午',
                  morning: '早',
                  afternoon: '下午',
                  evening: '晚',
                  night: '夜',
                },
                abbreviated: {
                  am: '上午',
                  pm: '下午',
                  midnight: '凌晨',
                  noon: '中午',
                  morning: '早晨',
                  afternoon: '中午',
                  evening: '晚上',
                  night: '夜間',
                },
                wide: {
                  am: '上午',
                  pm: '下午',
                  midnight: '凌晨',
                  noon: '中午',
                  morning: '早晨',
                  afternoon: '中午',
                  evening: '晚上',
                  night: '夜間',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: '上',
                  pm: '下',
                  midnight: '凌晨',
                  noon: '午',
                  morning: '早',
                  afternoon: '下午',
                  evening: '晚',
                  night: '夜',
                },
                abbreviated: {
                  am: '上午',
                  pm: '下午',
                  midnight: '凌晨',
                  noon: '中午',
                  morning: '早晨',
                  afternoon: '中午',
                  evening: '晚上',
                  night: '夜間',
                },
                wide: {
                  am: '上午',
                  pm: '下午',
                  midnight: '凌晨',
                  noon: '中午',
                  morning: '早晨',
                  afternoon: '中午',
                  evening: '晚上',
                  night: '夜間',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/match/index.js':
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
              matchPattern: /^(第\s*)?\d+(日|時|分|秒)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: { narrow: /^(前)/i, abbreviated: /^(前)/i, wide: /^(公元前|公元)/i },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^(前)/i, /^(公元)/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^第[一二三四]刻/i,
                wide: /^第[一二三四]刻鐘/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/(1|一)/i, /(2|二)/i, /(3|三)/i, /(4|四)/i] },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^(一|二|三|四|五|六|七|八|九|十[二一])/i,
                abbreviated: /^(一|二|三|四|五|六|七|八|九|十[二一]|\d|1[12])月/i,
                wide: /^(一|二|三|四|五|六|七|八|九|十[二一])月/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^一/i,
                  /^二/i,
                  /^三/i,
                  /^四/i,
                  /^五/i,
                  /^六/i,
                  /^七/i,
                  /^八/i,
                  /^九/i,
                  /^十(?!(一|二))/i,
                  /^十一/i,
                  /^十二/i,
                ],
                any: [
                  /^一|1/i,
                  /^二|2/i,
                  /^三|3/i,
                  /^四|4/i,
                  /^五|5/i,
                  /^六|6/i,
                  /^七|7/i,
                  /^八|8/i,
                  /^九|9/i,
                  /^十(?!(一|二))|10/i,
                  /^十一|11/i,
                  /^十二|12/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[一二三四五六日]/i,
                short: /^[一二三四五六日]/i,
                abbreviated: /^週[一二三四五六日]/i,
                wide: /^星期[一二三四五六日]/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/日/i, /一/i, /二/i, /三/i, /四/i, /五/i, /六/i] },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: { any: /^(上午?|下午?|午夜|[中正]午|早上?|下午|晚上?|凌晨)/i },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^上午?/i,
                  pm: /^下午?/i,
                  midnight: /^午夜/i,
                  noon: /^[中正]午/i,
                  morning: /^早上/i,
                  afternoon: /^下午/i,
                  evening: /^晚上?/i,
                  night: /^凌晨/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'zh-TW',
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
