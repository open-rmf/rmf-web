(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [75261, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/localize/index.js':
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
              return rem100 < 10 && rem100 % 10 == 1 ? number + '֊ին' : number + '֊րդ';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['Ք', 'Մ'],
                abbreviated: ['ՔԱ', 'ՄԹ'],
                wide: ['Քրիստոսից առաջ', 'Մեր թվարկության'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Ք1', 'Ք2', 'Ք3', 'Ք4'],
                wide: ['1֊ին քառորդ', '2֊րդ քառորդ', '3֊րդ քառորդ', '4֊րդ քառորդ'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['Հ', 'Փ', 'Մ', 'Ա', 'Մ', 'Հ', 'Հ', 'Օ', 'Ս', 'Հ', 'Ն', 'Դ'],
                abbreviated: [
                  'հուն',
                  'փետ',
                  'մար',
                  'ապր',
                  'մայ',
                  'հուն',
                  'հուլ',
                  'օգս',
                  'սեպ',
                  'հոկ',
                  'նոյ',
                  'դեկ',
                ],
                wide: [
                  'հունվար',
                  'փետրվար',
                  'մարտ',
                  'ապրիլ',
                  'մայիս',
                  'հունիս',
                  'հուլիս',
                  'օգոստոս',
                  'սեպտեմբեր',
                  'հոկտեմբեր',
                  'նոյեմբեր',
                  'դեկտեմբեր',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Կ', 'Ե', 'Ե', 'Չ', 'Հ', 'Ո', 'Շ'],
                short: ['կր', 'եր', 'եք', 'չք', 'հգ', 'ուր', 'շբ'],
                abbreviated: ['կիր', 'երկ', 'երք', 'չոր', 'հնգ', 'ուրբ', 'շաբ'],
                wide: [
                  'կիրակի',
                  'երկուշաբթի',
                  'երեքշաբթի',
                  'չորեքշաբթի',
                  'հինգշաբթի',
                  'ուրբաթ',
                  'շաբաթ',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'կեսգշ',
                  noon: 'կեսօր',
                  morning: 'առավոտ',
                  afternoon: 'ցերեկ',
                  evening: 'երեկո',
                  night: 'գիշեր',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'կեսգիշեր',
                  noon: 'կեսօր',
                  morning: 'առավոտ',
                  afternoon: 'ցերեկ',
                  evening: 'երեկո',
                  night: 'գիշեր',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'կեսգիշեր',
                  noon: 'կեսօր',
                  morning: 'առավոտ',
                  afternoon: 'ցերեկ',
                  evening: 'երեկո',
                  night: 'գիշեր',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'կեսգշ',
                  noon: 'կեսօր',
                  morning: 'առավոտը',
                  afternoon: 'ցերեկը',
                  evening: 'երեկոյան',
                  night: 'գիշերը',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'կեսգիշերին',
                  noon: 'կեսօրին',
                  morning: 'առավոտը',
                  afternoon: 'ցերեկը',
                  evening: 'երեկոյան',
                  night: 'գիշերը',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'կեսգիշերին',
                  noon: 'կեսօրին',
                  morning: 'առավոտը',
                  afternoon: 'ցերեկը',
                  evening: 'երեկոյան',
                  night: 'գիշերը',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
