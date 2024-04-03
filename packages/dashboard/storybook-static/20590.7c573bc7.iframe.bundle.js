(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [20590, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/_lib/localize/index.js':
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
              var unit = String(null == options ? void 0 : options.unit),
                number = Number(dirtyNumber);
              return (
                number +
                ('date' === unit
                  ? 3 === number || 23 === number
                    ? '-є'
                    : '-е'
                  : 'minute' === unit || 'second' === unit || 'hour' === unit
                    ? '-а'
                    : '-й')
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['до н.е.', 'н.е.'],
                abbreviated: ['до н. е.', 'н. е.'],
                wide: ['до нашої ери', 'нашої ери'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-й кв.', '2-й кв.', '3-й кв.', '4-й кв.'],
                wide: ['1-й квартал', '2-й квартал', '3-й квартал', '4-й квартал'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['С', 'Л', 'Б', 'К', 'Т', 'Ч', 'Л', 'С', 'В', 'Ж', 'Л', 'Г'],
                abbreviated: [
                  'січ.',
                  'лют.',
                  'берез.',
                  'квіт.',
                  'трав.',
                  'черв.',
                  'лип.',
                  'серп.',
                  'верес.',
                  'жовт.',
                  'листоп.',
                  'груд.',
                ],
                wide: [
                  'січень',
                  'лютий',
                  'березень',
                  'квітень',
                  'травень',
                  'червень',
                  'липень',
                  'серпень',
                  'вересень',
                  'жовтень',
                  'листопад',
                  'грудень',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['С', 'Л', 'Б', 'К', 'Т', 'Ч', 'Л', 'С', 'В', 'Ж', 'Л', 'Г'],
                abbreviated: [
                  'січ.',
                  'лют.',
                  'берез.',
                  'квіт.',
                  'трав.',
                  'черв.',
                  'лип.',
                  'серп.',
                  'верес.',
                  'жовт.',
                  'листоп.',
                  'груд.',
                ],
                wide: [
                  'січня',
                  'лютого',
                  'березня',
                  'квітня',
                  'травня',
                  'червня',
                  'липня',
                  'серпня',
                  'вересня',
                  'жовтня',
                  'листопада',
                  'грудня',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Н', 'П', 'В', 'С', 'Ч', 'П', 'С'],
                short: ['нд', 'пн', 'вт', 'ср', 'чт', 'пт', 'сб'],
                abbreviated: ['нед', 'пон', 'вів', 'сер', 'чтв', 'птн', 'суб'],
                wide: ['неділя', 'понеділок', 'вівторок', 'середа', 'четвер', 'п’ятниця', 'субота'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'півн.',
                  noon: 'пол.',
                  morning: 'ранок',
                  afternoon: 'день',
                  evening: 'веч.',
                  night: 'ніч',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'півн.',
                  noon: 'пол.',
                  morning: 'ранок',
                  afternoon: 'день',
                  evening: 'веч.',
                  night: 'ніч',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'північ',
                  noon: 'полудень',
                  morning: 'ранок',
                  afternoon: 'день',
                  evening: 'вечір',
                  night: 'ніч',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'півн.',
                  noon: 'пол.',
                  morning: 'ранку',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночі',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'півн.',
                  noon: 'пол.',
                  morning: 'ранку',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночі',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'північ',
                  noon: 'полудень',
                  morning: 'ранку',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночі',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
