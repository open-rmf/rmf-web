(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [44281, 69107],
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
  },
]);
