(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [41408, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/localize/index.js':
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
                  ? '-га'
                  : 'hour' === unit || 'minute' === unit || 'second' === unit
                    ? '-я'
                    : (number % 10 != 2 && number % 10 != 3) ||
                        number % 100 == 12 ||
                        number % 100 == 13
                      ? '-ы'
                      : '-і')
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['да н.э.', 'н.э.'],
                abbreviated: ['да н. э.', 'н. э.'],
                wide: ['да нашай эры', 'нашай эры'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-ы кв.', '2-і кв.', '3-і кв.', '4-ы кв.'],
                wide: ['1-ы квартал', '2-і квартал', '3-і квартал', '4-ы квартал'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['С', 'Л', 'С', 'К', 'Т', 'Ч', 'Л', 'Ж', 'В', 'К', 'Л', 'С'],
                abbreviated: [
                  'студз.',
                  'лют.',
                  'сак.',
                  'крас.',
                  'трав.',
                  'чэрв.',
                  'ліп.',
                  'жн.',
                  'вер.',
                  'кастр.',
                  'ліст.',
                  'сьнеж.',
                ],
                wide: [
                  'студзень',
                  'люты',
                  'сакавік',
                  'красавік',
                  'травень',
                  'чэрвень',
                  'ліпень',
                  'жнівень',
                  'верасень',
                  'кастрычнік',
                  'лістапад',
                  'сьнежань',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['С', 'Л', 'С', 'К', 'Т', 'Ч', 'Л', 'Ж', 'В', 'К', 'Л', 'С'],
                abbreviated: [
                  'студз.',
                  'лют.',
                  'сак.',
                  'крас.',
                  'трав.',
                  'чэрв.',
                  'ліп.',
                  'жн.',
                  'вер.',
                  'кастр.',
                  'ліст.',
                  'сьнеж.',
                ],
                wide: [
                  'студзеня',
                  'лютага',
                  'сакавіка',
                  'красавіка',
                  'траўня',
                  'чэрвеня',
                  'ліпеня',
                  'жніўня',
                  'верасня',
                  'кастрычніка',
                  'лістапада',
                  'сьнежня',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Н', 'П', 'А', 'С', 'Ч', 'П', 'С'],
                short: ['нд', 'пн', 'аў', 'ср', 'чц', 'пт', 'сб'],
                abbreviated: ['нядз', 'пан', 'аўт', 'сер', 'чаць', 'пят', 'суб'],
                wide: [
                  'нядзеля',
                  'панядзелак',
                  'аўторак',
                  'серада',
                  'чацьвер',
                  'пятніца',
                  'субота',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дзень',
                  evening: 'веч.',
                  night: 'ноч',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дзень',
                  evening: 'веч.',
                  night: 'ноч',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўнач',
                  noon: 'поўдзень',
                  morning: 'раніца',
                  afternoon: 'дзень',
                  evening: 'вечар',
                  night: 'ноч',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночы',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночы',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўнач',
                  noon: 'поўдзень',
                  morning: 'раніцы',
                  afternoon: 'дня',
                  evening: 'вечара',
                  night: 'ночы',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
