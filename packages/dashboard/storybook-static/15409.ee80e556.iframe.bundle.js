(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [15409, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/localize/index.js':
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
        );
        function numberWithSuffix(number, unit, masculine, feminine, neuter) {
          var suffix = (function isNeuter(unit) {
            return 'quarter' === unit;
          })(unit)
            ? neuter
            : (function isFeminine(unit) {
                  return (
                    'year' === unit || 'week' === unit || 'minute' === unit || 'second' === unit
                  );
                })(unit)
              ? feminine
              : masculine;
          return number + '-' + suffix;
        }
        var _default = {
          ordinalNumber: function ordinalNumber(dirtyNumber, options) {
            var number = Number(dirtyNumber),
              unit = null == options ? void 0 : options.unit;
            if (0 === number) return numberWithSuffix(0, unit, 'ев', 'ева', 'ево');
            if (number % 1e3 == 0) return numberWithSuffix(number, unit, 'ен', 'на', 'но');
            if (number % 100 == 0) return numberWithSuffix(number, unit, 'тен', 'тна', 'тно');
            var rem100 = number % 100;
            if (rem100 > 20 || rem100 < 10)
              switch (rem100 % 10) {
                case 1:
                  return numberWithSuffix(number, unit, 'ви', 'ва', 'во');
                case 2:
                  return numberWithSuffix(number, unit, 'ри', 'ра', 'ро');
                case 7:
                case 8:
                  return numberWithSuffix(number, unit, 'ми', 'ма', 'мо');
              }
            return numberWithSuffix(number, unit, 'ти', 'та', 'то');
          },
          era: (0, _index.default)({
            values: {
              narrow: ['пр.н.е.', 'н.е.'],
              abbreviated: ['преди н. е.', 'н. е.'],
              wide: ['преди новата ера', 'новата ера'],
            },
            defaultWidth: 'wide',
          }),
          quarter: (0, _index.default)({
            values: {
              narrow: ['1', '2', '3', '4'],
              abbreviated: ['1-во тримес.', '2-ро тримес.', '3-то тримес.', '4-то тримес.'],
              wide: ['1-во тримесечие', '2-ро тримесечие', '3-то тримесечие', '4-то тримесечие'],
            },
            defaultWidth: 'wide',
            argumentCallback: function argumentCallback(quarter) {
              return quarter - 1;
            },
          }),
          month: (0, _index.default)({
            values: {
              abbreviated: [
                'яну',
                'фев',
                'мар',
                'апр',
                'май',
                'юни',
                'юли',
                'авг',
                'сеп',
                'окт',
                'ное',
                'дек',
              ],
              wide: [
                'януари',
                'февруари',
                'март',
                'април',
                'май',
                'юни',
                'юли',
                'август',
                'септември',
                'октомври',
                'ноември',
                'декември',
              ],
            },
            defaultWidth: 'wide',
          }),
          day: (0, _index.default)({
            values: {
              narrow: ['Н', 'П', 'В', 'С', 'Ч', 'П', 'С'],
              short: ['нд', 'пн', 'вт', 'ср', 'чт', 'пт', 'сб'],
              abbreviated: ['нед', 'пон', 'вто', 'сря', 'чет', 'пет', 'съб'],
              wide: ['неделя', 'понеделник', 'вторник', 'сряда', 'четвъртък', 'петък', 'събота'],
            },
            defaultWidth: 'wide',
          }),
          dayPeriod: (0, _index.default)({
            values: {
              wide: {
                am: 'преди обяд',
                pm: 'след обяд',
                midnight: 'в полунощ',
                noon: 'на обяд',
                morning: 'сутринта',
                afternoon: 'следобед',
                evening: 'вечерта',
                night: 'през нощта',
              },
            },
            defaultWidth: 'wide',
          }),
        };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
