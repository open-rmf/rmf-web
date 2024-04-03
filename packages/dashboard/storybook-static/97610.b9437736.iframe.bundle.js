(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [97610, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/localize/index.js':
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
              return Number(dirtyNumber) + '.';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['f.Kr.', 'e.Kr.'],
                abbreviated: ['f.Kr.', 'e.Kr.'],
                wide: ['fyrir Krist', 'eftir Krist'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1F', '2F', '3F', '4F'],
                wide: ['1. fjórðungur', '2. fjórðungur', '3. fjórðungur', '4. fjórðungur'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['J', 'F', 'M', 'A', 'M', 'J', 'J', 'Á', 'S', 'Ó', 'N', 'D'],
                abbreviated: [
                  'jan.',
                  'feb.',
                  'mars',
                  'apríl',
                  'maí',
                  'júní',
                  'júlí',
                  'ágúst',
                  'sept.',
                  'okt.',
                  'nóv.',
                  'des.',
                ],
                wide: [
                  'janúar',
                  'febrúar',
                  'mars',
                  'apríl',
                  'maí',
                  'júní',
                  'júlí',
                  'ágúst',
                  'september',
                  'október',
                  'nóvember',
                  'desember',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['S', 'M', 'Þ', 'M', 'F', 'F', 'L'],
                short: ['Su', 'Má', 'Þr', 'Mi', 'Fi', 'Fö', 'La'],
                abbreviated: ['sun.', 'mán.', 'þri.', 'mið.', 'fim.', 'fös.', 'lau.'],
                wide: [
                  'sunnudagur',
                  'mánudagur',
                  'þriðjudagur',
                  'miðvikudagur',
                  'fimmtudagur',
                  'föstudagur',
                  'laugardagur',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'f',
                  pm: 'e',
                  midnight: 'miðnætti',
                  noon: 'hádegi',
                  morning: 'morgunn',
                  afternoon: 'síðdegi',
                  evening: 'kvöld',
                  night: 'nótt',
                },
                abbreviated: {
                  am: 'f.h.',
                  pm: 'e.h.',
                  midnight: 'miðnætti',
                  noon: 'hádegi',
                  morning: 'morgunn',
                  afternoon: 'síðdegi',
                  evening: 'kvöld',
                  night: 'nótt',
                },
                wide: {
                  am: 'fyrir hádegi',
                  pm: 'eftir hádegi',
                  midnight: 'miðnætti',
                  noon: 'hádegi',
                  morning: 'morgunn',
                  afternoon: 'síðdegi',
                  evening: 'kvöld',
                  night: 'nótt',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'f',
                  pm: 'e',
                  midnight: 'á miðnætti',
                  noon: 'á hádegi',
                  morning: 'að morgni',
                  afternoon: 'síðdegis',
                  evening: 'um kvöld',
                  night: 'um nótt',
                },
                abbreviated: {
                  am: 'f.h.',
                  pm: 'e.h.',
                  midnight: 'á miðnætti',
                  noon: 'á hádegi',
                  morning: 'að morgni',
                  afternoon: 'síðdegis',
                  evening: 'um kvöld',
                  night: 'um nótt',
                },
                wide: {
                  am: 'fyrir hádegi',
                  pm: 'eftir hádegi',
                  midnight: 'á miðnætti',
                  noon: 'á hádegi',
                  morning: 'að morgni',
                  afternoon: 'síðdegis',
                  evening: 'um kvöld',
                  night: 'um nótt',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
