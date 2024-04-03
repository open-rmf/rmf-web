(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [28634, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/localize/index.js':
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
            ordinalNumber: function ordinalNumber(dirtyNumber, _) {
              return Number(dirtyNumber).toString();
            },
            era: (0, _index.default)({
              values: {
                narrow: ['ម.គស', 'គស'],
                abbreviated: ['មុនគ.ស', 'គ.ស'],
                wide: ['មុនគ្រិស្តសករាជ', 'នៃគ្រិស្តសករាជ'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['ត្រីមាសទី 1', 'ត្រីមាសទី 2', 'ត្រីមាសទី 3', 'ត្រីមាសទី 4'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: [
                  'ម.ក',
                  'ក.ម',
                  'មិ',
                  'ម.ស',
                  'ឧ.ស',
                  'ម.ថ',
                  'ក.ដ',
                  'សី',
                  'កញ',
                  'តុ',
                  'វិ',
                  'ធ',
                ],
                abbreviated: [
                  'មករា',
                  'កុម្ភៈ',
                  'មីនា',
                  'មេសា',
                  'ឧសភា',
                  'មិថុនា',
                  'កក្កដា',
                  'សីហា',
                  'កញ្ញា',
                  'តុលា',
                  'វិច្ឆិកា',
                  'ធ្នូ',
                ],
                wide: [
                  'មករា',
                  'កុម្ភៈ',
                  'មីនា',
                  'មេសា',
                  'ឧសភា',
                  'មិថុនា',
                  'កក្កដា',
                  'សីហា',
                  'កញ្ញា',
                  'តុលា',
                  'វិច្ឆិកា',
                  'ធ្នូ',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['អា', 'ច', 'អ', 'ព', 'ព្រ', 'សុ', 'ស'],
                short: ['អា', 'ច', 'អ', 'ព', 'ព្រ', 'សុ', 'ស'],
                abbreviated: ['អា', 'ច', 'អ', 'ព', 'ព្រ', 'សុ', 'ស'],
                wide: ['អាទិត្យ', 'ចន្ទ', 'អង្គារ', 'ពុធ', 'ព្រហស្បតិ៍', 'សុក្រ', 'សៅរ៍'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ព្រឹក',
                  pm: 'ល្ងាច',
                  midnight: '​ពេលកណ្ដាលអធ្រាត្រ',
                  noon: 'ពេលថ្ងៃត្រង់',
                  morning: 'ពេលព្រឹក',
                  afternoon: 'ពេលរសៀល',
                  evening: 'ពេលល្ងាច',
                  night: 'ពេលយប់',
                },
                abbreviated: {
                  am: 'ព្រឹក',
                  pm: 'ល្ងាច',
                  midnight: '​ពេលកណ្ដាលអធ្រាត្រ',
                  noon: 'ពេលថ្ងៃត្រង់',
                  morning: 'ពេលព្រឹក',
                  afternoon: 'ពេលរសៀល',
                  evening: 'ពេលល្ងាច',
                  night: 'ពេលយប់',
                },
                wide: {
                  am: 'ព្រឹក',
                  pm: 'ល្ងាច',
                  midnight: '​ពេលកណ្ដាលអធ្រាត្រ',
                  noon: 'ពេលថ្ងៃត្រង់',
                  morning: 'ពេលព្រឹក',
                  afternoon: 'ពេលរសៀល',
                  evening: 'ពេលល្ងាច',
                  night: 'ពេលយប់',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'ព្រឹក',
                  pm: 'ល្ងាច',
                  midnight: '​ពេលកណ្ដាលអធ្រាត្រ',
                  noon: 'ពេលថ្ងៃត្រង់',
                  morning: 'ពេលព្រឹក',
                  afternoon: 'ពេលរសៀល',
                  evening: 'ពេលល្ងាច',
                  night: 'ពេលយប់',
                },
                abbreviated: {
                  am: 'ព្រឹក',
                  pm: 'ល្ងាច',
                  midnight: '​ពេលកណ្ដាលអធ្រាត្រ',
                  noon: 'ពេលថ្ងៃត្រង់',
                  morning: 'ពេលព្រឹក',
                  afternoon: 'ពេលរសៀល',
                  evening: 'ពេលល្ងាច',
                  night: 'ពេលយប់',
                },
                wide: {
                  am: 'ព្រឹក',
                  pm: 'ល្ងាច',
                  midnight: '​ពេលកណ្ដាលអធ្រាត្រ',
                  noon: 'ពេលថ្ងៃត្រង់',
                  morning: 'ពេលព្រឹក',
                  afternoon: 'ពេលរសៀល',
                  evening: 'ពេលល្ងាច',
                  night: 'ពេលយប់',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
