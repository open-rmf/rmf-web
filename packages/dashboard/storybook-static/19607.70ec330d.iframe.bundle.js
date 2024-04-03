(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [19607, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/localize/index.js':
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
              return 'week' === (null == options ? void 0 : options.unit)
                ? number + 'ª'
                : number + 'º';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['AC', 'DC'],
                abbreviated: ['AC', 'DC'],
                wide: ['antes de cristo', 'depois de cristo'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['T1', 'T2', 'T3', 'T4'],
                wide: ['1º trimestre', '2º trimestre', '3º trimestre', '4º trimestre'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['j', 'f', 'm', 'a', 'm', 'j', 'j', 'a', 's', 'o', 'n', 'd'],
                abbreviated: [
                  'jan',
                  'fev',
                  'mar',
                  'abr',
                  'mai',
                  'jun',
                  'jul',
                  'ago',
                  'set',
                  'out',
                  'nov',
                  'dez',
                ],
                wide: [
                  'janeiro',
                  'fevereiro',
                  'março',
                  'abril',
                  'maio',
                  'junho',
                  'julho',
                  'agosto',
                  'setembro',
                  'outubro',
                  'novembro',
                  'dezembro',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['D', 'S', 'T', 'Q', 'Q', 'S', 'S'],
                short: ['dom', 'seg', 'ter', 'qua', 'qui', 'sex', 'sab'],
                abbreviated: ['domingo', 'segunda', 'terça', 'quarta', 'quinta', 'sexta', 'sábado'],
                wide: [
                  'domingo',
                  'segunda-feira',
                  'terça-feira',
                  'quarta-feira',
                  'quinta-feira',
                  'sexta-feira',
                  'sábado',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'mn',
                  noon: 'md',
                  morning: 'manhã',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noite',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'manhã',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noite',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'manhã',
                  afternoon: 'tarde',
                  evening: 'tarde',
                  night: 'noite',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'mn',
                  noon: 'md',
                  morning: 'da manhã',
                  afternoon: 'da tarde',
                  evening: 'da tarde',
                  night: 'da noite',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'da manhã',
                  afternoon: 'da tarde',
                  evening: 'da tarde',
                  night: 'da noite',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'meia-noite',
                  noon: 'meio-dia',
                  morning: 'da manhã',
                  afternoon: 'da tarde',
                  evening: 'da tarde',
                  night: 'da noite',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
