(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [70870, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/localize/index.js':
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
                    return number + '-ви';
                  case 2:
                    return number + '-ри';
                  case 7:
                  case 8:
                    return number + '-ми';
                }
              return number + '-ти';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['пр.н.е.', 'н.е.'],
                abbreviated: ['пред н. е.', 'н. е.'],
                wide: ['пред нашата ера', 'нашата ера'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-ви кв.', '2-ри кв.', '3-ти кв.', '4-ти кв.'],
                wide: ['1-ви квартал', '2-ри квартал', '3-ти квартал', '4-ти квартал'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                abbreviated: [
                  'јан',
                  'фев',
                  'мар',
                  'апр',
                  'мај',
                  'јун',
                  'јул',
                  'авг',
                  'септ',
                  'окт',
                  'ноем',
                  'дек',
                ],
                wide: [
                  'јануари',
                  'февруари',
                  'март',
                  'април',
                  'мај',
                  'јуни',
                  'јули',
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
                short: ['не', 'по', 'вт', 'ср', 'че', 'пе', 'са'],
                abbreviated: ['нед', 'пон', 'вто', 'сре', 'чет', 'пет', 'саб'],
                wide: ['недела', 'понеделник', 'вторник', 'среда', 'четврток', 'петок', 'сабота'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                wide: {
                  am: 'претпладне',
                  pm: 'попладне',
                  midnight: 'полноќ',
                  noon: 'напладне',
                  morning: 'наутро',
                  afternoon: 'попладне',
                  evening: 'навечер',
                  night: 'ноќе',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
