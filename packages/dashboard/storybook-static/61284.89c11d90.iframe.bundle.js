(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [61284, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kk/_lib/localize/index.js':
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
          suffixes = {
            0: '-ші',
            1: '-ші',
            2: '-ші',
            3: '-ші',
            4: '-ші',
            5: '-ші',
            6: '-шы',
            7: '-ші',
            8: '-ші',
            9: '-шы',
            10: '-шы',
            20: '-шы',
            30: '-шы',
            40: '-шы',
            50: '-ші',
            60: '-шы',
            70: '-ші',
            80: '-ші',
            90: '-шы',
            100: '-ші',
          },
          _default = {
            ordinalNumber: function ordinalNumber(dirtyNumber, _options) {
              var number = Number(dirtyNumber),
                b = number >= 100 ? 100 : null;
              return (
                number + (suffixes[number] || suffixes[number % 10] || (b && suffixes[b]) || '')
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['б.з.д.', 'б.з.'],
                abbreviated: ['б.з.д.', 'б.з.'],
                wide: ['біздің заманымызға дейін', 'біздің заманымыз'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-ші тоқ.', '2-ші тоқ.', '3-ші тоқ.', '4-ші тоқ.'],
                wide: ['1-ші тоқсан', '2-ші тоқсан', '3-ші тоқсан', '4-ші тоқсан'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['Қ', 'А', 'Н', 'С', 'М', 'М', 'Ш', 'Т', 'Қ', 'Қ', 'Қ', 'Ж'],
                abbreviated: [
                  'қаң',
                  'ақп',
                  'нау',
                  'сәу',
                  'мам',
                  'мау',
                  'шіл',
                  'там',
                  'қыр',
                  'қаз',
                  'қар',
                  'жел',
                ],
                wide: [
                  'қаңтар',
                  'ақпан',
                  'наурыз',
                  'сәуір',
                  'мамыр',
                  'маусым',
                  'шілде',
                  'тамыз',
                  'қыркүйек',
                  'қазан',
                  'қараша',
                  'желтоқсан',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['Қ', 'А', 'Н', 'С', 'М', 'М', 'Ш', 'Т', 'Қ', 'Қ', 'Қ', 'Ж'],
                abbreviated: [
                  'қаң',
                  'ақп',
                  'нау',
                  'сәу',
                  'мам',
                  'мау',
                  'шіл',
                  'там',
                  'қыр',
                  'қаз',
                  'қар',
                  'жел',
                ],
                wide: [
                  'қаңтар',
                  'ақпан',
                  'наурыз',
                  'сәуір',
                  'мамыр',
                  'маусым',
                  'шілде',
                  'тамыз',
                  'қыркүйек',
                  'қазан',
                  'қараша',
                  'желтоқсан',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Ж', 'Д', 'С', 'С', 'Б', 'Ж', 'С'],
                short: ['жс', 'дс', 'сс', 'ср', 'бс', 'жм', 'сб'],
                abbreviated: ['жс', 'дс', 'сс', 'ср', 'бс', 'жм', 'сб'],
                wide: ['жексенбі', 'дүйсенбі', 'сейсенбі', 'сәрсенбі', 'бейсенбі', 'жұма', 'сенбі'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасы',
                  noon: 'түс',
                  morning: 'таң',
                  afternoon: 'күндіз',
                  evening: 'кеш',
                  night: 'түн',
                },
                wide: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасы',
                  noon: 'түс',
                  morning: 'таң',
                  afternoon: 'күндіз',
                  evening: 'кеш',
                  night: 'түн',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                narrow: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасында',
                  noon: 'түс',
                  morning: 'таң',
                  afternoon: 'күн',
                  evening: 'кеш',
                  night: 'түн',
                },
                wide: {
                  am: 'ТД',
                  pm: 'ТК',
                  midnight: 'түн ортасында',
                  noon: 'түсте',
                  morning: 'таңертең',
                  afternoon: 'күндіз',
                  evening: 'кеште',
                  night: 'түнде',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
