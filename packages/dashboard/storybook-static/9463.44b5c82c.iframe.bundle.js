(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [9463, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/localize/index.js':
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
              var number = Number(dirtyNumber),
                unit = null == options ? void 0 : options.unit;
              return (
                number +
                ('date' === unit
                  ? '-е'
                  : 'week' === unit || 'minute' === unit || 'second' === unit
                    ? '-я'
                    : '-й')
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['до н.э.', 'н.э.'],
                abbreviated: ['до н. э.', 'н. э.'],
                wide: ['до нашей эры', 'нашей эры'],
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
                narrow: ['Я', 'Ф', 'М', 'А', 'М', 'И', 'И', 'А', 'С', 'О', 'Н', 'Д'],
                abbreviated: [
                  'янв.',
                  'фев.',
                  'март',
                  'апр.',
                  'май',
                  'июнь',
                  'июль',
                  'авг.',
                  'сент.',
                  'окт.',
                  'нояб.',
                  'дек.',
                ],
                wide: [
                  'январь',
                  'февраль',
                  'март',
                  'апрель',
                  'май',
                  'июнь',
                  'июль',
                  'август',
                  'сентябрь',
                  'октябрь',
                  'ноябрь',
                  'декабрь',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['Я', 'Ф', 'М', 'А', 'М', 'И', 'И', 'А', 'С', 'О', 'Н', 'Д'],
                abbreviated: [
                  'янв.',
                  'фев.',
                  'мар.',
                  'апр.',
                  'мая',
                  'июн.',
                  'июл.',
                  'авг.',
                  'сент.',
                  'окт.',
                  'нояб.',
                  'дек.',
                ],
                wide: [
                  'января',
                  'февраля',
                  'марта',
                  'апреля',
                  'мая',
                  'июня',
                  'июля',
                  'августа',
                  'сентября',
                  'октября',
                  'ноября',
                  'декабря',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['В', 'П', 'В', 'С', 'Ч', 'П', 'С'],
                short: ['вс', 'пн', 'вт', 'ср', 'чт', 'пт', 'сб'],
                abbreviated: ['вск', 'пнд', 'втр', 'срд', 'чтв', 'птн', 'суб'],
                wide: [
                  'воскресенье',
                  'понедельник',
                  'вторник',
                  'среда',
                  'четверг',
                  'пятница',
                  'суббота',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утро',
                  afternoon: 'день',
                  evening: 'веч.',
                  night: 'ночь',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утро',
                  afternoon: 'день',
                  evening: 'веч.',
                  night: 'ночь',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полночь',
                  noon: 'полдень',
                  morning: 'утро',
                  afternoon: 'день',
                  evening: 'вечер',
                  night: 'ночь',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утра',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночи',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полн.',
                  noon: 'полд.',
                  morning: 'утра',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночи',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'полночь',
                  noon: 'полдень',
                  morning: 'утра',
                  afternoon: 'дня',
                  evening: 'вечера',
                  night: 'ночи',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
