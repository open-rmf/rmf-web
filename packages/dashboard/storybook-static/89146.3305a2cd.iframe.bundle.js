(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [89146, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/_lib/localize/index.js':
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
                wide: ['før Kristus', 'etter Kristus'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1. kvartal', '2. kvartal', '3. kvartal', '4. kvartal'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['J', 'F', 'M', 'A', 'M', 'J', 'J', 'A', 'S', 'O', 'N', 'D'],
                abbreviated: [
                  'jan.',
                  'feb.',
                  'mars',
                  'apr.',
                  'mai',
                  'juni',
                  'juli',
                  'aug.',
                  'sep.',
                  'okt.',
                  'nov.',
                  'des.',
                ],
                wide: [
                  'januar',
                  'februar',
                  'mars',
                  'april',
                  'mai',
                  'juni',
                  'juli',
                  'august',
                  'september',
                  'oktober',
                  'november',
                  'desember',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['S', 'M', 'T', 'O', 'T', 'F', 'L'],
                short: ['sø', 'ma', 'ti', 'on', 'to', 'fr', 'lø'],
                abbreviated: ['søn', 'man', 'tir', 'ons', 'tor', 'fre', 'lør'],
                wide: ['søndag', 'mandag', 'tirsdag', 'onsdag', 'torsdag', 'fredag', 'lørdag'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'a',
                  pm: 'p',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'på morg.',
                  afternoon: 'på etterm.',
                  evening: 'på kvelden',
                  night: 'på natten',
                },
                abbreviated: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'på morg.',
                  afternoon: 'på etterm.',
                  evening: 'på kvelden',
                  night: 'på natten',
                },
                wide: {
                  am: 'a.m.',
                  pm: 'p.m.',
                  midnight: 'midnatt',
                  noon: 'middag',
                  morning: 'på morgenen',
                  afternoon: 'på ettermiddagen',
                  evening: 'på kvelden',
                  night: 'på natten',
                },
              },
              defaultWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
