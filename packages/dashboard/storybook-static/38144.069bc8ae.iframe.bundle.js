(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [38144, 69107],
  {
    '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js':
      (module) => {
        (module.exports = function _interopRequireDefault(obj) {
          return obj && obj.__esModule ? obj : { default: obj };
        }),
          (module.exports.__esModule = !0),
          (module.exports.default = module.exports);
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
  },
]);
