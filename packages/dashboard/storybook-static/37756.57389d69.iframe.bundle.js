(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [37756, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/localize/index.js':
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
                narrow: ['pred Kr.', 'po Kr.'],
                abbreviated: ['pred Kr.', 'po Kr.'],
                wide: ['pred Kristom', 'po Kristovi'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1. štvrťrok', '2. štvrťrok', '3. štvrťrok', '4. štvrťrok'],
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
                  'feb',
                  'mar',
                  'apr',
                  'máj',
                  'jún',
                  'júl',
                  'aug',
                  'sep',
                  'okt',
                  'nov',
                  'dec',
                ],
                wide: [
                  'január',
                  'február',
                  'marec',
                  'apríl',
                  'máj',
                  'jún',
                  'júl',
                  'august',
                  'september',
                  'október',
                  'november',
                  'december',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['j', 'f', 'm', 'a', 'm', 'j', 'j', 'a', 's', 'o', 'n', 'd'],
                abbreviated: [
                  'jan',
                  'feb',
                  'mar',
                  'apr',
                  'máj',
                  'jún',
                  'júl',
                  'aug',
                  'sep',
                  'okt',
                  'nov',
                  'dec',
                ],
                wide: [
                  'januára',
                  'februára',
                  'marca',
                  'apríla',
                  'mája',
                  'júna',
                  'júla',
                  'augusta',
                  'septembra',
                  'októbra',
                  'novembra',
                  'decembra',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['n', 'p', 'u', 's', 'š', 'p', 's'],
                short: ['ne', 'po', 'ut', 'st', 'št', 'pi', 'so'],
                abbreviated: ['ne', 'po', 'ut', 'st', 'št', 'pi', 'so'],
                wide: ['nedeľa', 'pondelok', 'utorok', 'streda', 'štvrtok', 'piatok', 'sobota'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'poln.',
                  noon: 'pol.',
                  morning: 'ráno',
                  afternoon: 'pop.',
                  evening: 'več.',
                  night: 'noc',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'poln.',
                  noon: 'pol.',
                  morning: 'ráno',
                  afternoon: 'popol.',
                  evening: 'večer',
                  night: 'noc',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'polnoc',
                  noon: 'poludnie',
                  morning: 'ráno',
                  afternoon: 'popoludnie',
                  evening: 'večer',
                  night: 'noc',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'o poln.',
                  noon: 'nap.',
                  morning: 'ráno',
                  afternoon: 'pop.',
                  evening: 'več.',
                  night: 'v n.',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'o poln.',
                  noon: 'napol.',
                  morning: 'ráno',
                  afternoon: 'popol.',
                  evening: 'večer',
                  night: 'v noci',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'o polnoci',
                  noon: 'napoludnie',
                  morning: 'ráno',
                  afternoon: 'popoludní',
                  evening: 'večer',
                  night: 'v noci',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
