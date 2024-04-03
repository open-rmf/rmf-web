(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [69364, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/localize/index.js':
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
                narrow: ['М.А', 'М'],
                abbreviated: ['М.А', 'М'],
                wide: ['Милоддан Аввалги', 'Милодий'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-чор.', '2-чор.', '3-чор.', '4-чор.'],
                wide: ['1-чорак', '2-чорак', '3-чорак', '4-чорак'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['Я', 'Ф', 'М', 'А', 'М', 'И', 'И', 'А', 'С', 'О', 'Н', 'Д'],
                abbreviated: [
                  'янв',
                  'фев',
                  'мар',
                  'апр',
                  'май',
                  'июн',
                  'июл',
                  'авг',
                  'сен',
                  'окт',
                  'ноя',
                  'дек',
                ],
                wide: [
                  'январ',
                  'феврал',
                  'март',
                  'апрел',
                  'май',
                  'июн',
                  'июл',
                  'август',
                  'сентабр',
                  'октабр',
                  'ноябр',
                  'декабр',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Я', 'Д', 'С', 'Ч', 'П', 'Ж', 'Ш'],
                short: ['як', 'ду', 'се', 'чо', 'па', 'жу', 'ша'],
                abbreviated: ['якш', 'душ', 'сеш', 'чор', 'пай', 'жум', 'шан'],
                wide: ['якшанба', 'душанба', 'сешанба', 'чоршанба', 'пайшанба', 'жума', 'шанба'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                any: {
                  am: 'П.О.',
                  pm: 'П.К.',
                  midnight: 'ярим тун',
                  noon: 'пешин',
                  morning: 'эрталаб',
                  afternoon: 'пешиндан кейин',
                  evening: 'кечаси',
                  night: 'тун',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                any: {
                  am: 'П.О.',
                  pm: 'П.К.',
                  midnight: 'ярим тун',
                  noon: 'пешин',
                  morning: 'эрталаб',
                  afternoon: 'пешиндан кейин',
                  evening: 'кечаси',
                  night: 'тун',
                },
              },
              defaultFormattingWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
