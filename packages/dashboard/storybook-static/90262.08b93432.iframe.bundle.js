(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [90262, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/localize/index.js':
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
              return String(dirtyNumber);
            },
            era: (0, _index.default)({
              values: {
                narrow: ['ઈસપૂ', 'ઈસ'],
                abbreviated: ['ઈ.સ.પૂર્વે', 'ઈ.સ.'],
                wide: ['ઈસવીસન પૂર્વે', 'ઈસવીસન'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['Q1', 'Q2', 'Q3', 'Q4'],
                wide: ['1લો ત્રિમાસ', '2જો ત્રિમાસ', '3જો ત્રિમાસ', '4થો ત્રિમાસ'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['જા', 'ફે', 'મા', 'એ', 'મે', 'જૂ', 'જુ', 'ઓ', 'સ', 'ઓ', 'ન', 'ડિ'],
                abbreviated: [
                  'જાન્યુ',
                  'ફેબ્રુ',
                  'માર્ચ',
                  'એપ્રિલ',
                  'મે',
                  'જૂન',
                  'જુલાઈ',
                  'ઑગસ્ટ',
                  'સપ્ટે',
                  'ઓક્ટો',
                  'નવે',
                  'ડિસે',
                ],
                wide: [
                  'જાન્યુઆરી',
                  'ફેબ્રુઆરી',
                  'માર્ચ',
                  'એપ્રિલ',
                  'મે',
                  'જૂન',
                  'જુલાઇ',
                  'ઓગસ્ટ',
                  'સપ્ટેમ્બર',
                  'ઓક્ટોબર',
                  'નવેમ્બર',
                  'ડિસેમ્બર',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ર', 'સો', 'મં', 'બુ', 'ગુ', 'શુ', 'શ'],
                short: ['ર', 'સો', 'મં', 'બુ', 'ગુ', 'શુ', 'શ'],
                abbreviated: ['રવિ', 'સોમ', 'મંગળ', 'બુધ', 'ગુરુ', 'શુક્ર', 'શનિ'],
                wide: ['રવિવાર', 'સોમવાર', 'મંગળવાર', 'બુધવાર', 'ગુરુવાર', 'શુક્રવાર', 'શનિવાર'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'મ.રાત્રિ',
                  noon: 'બ.',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: '​મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: '​મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'મ.રાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                abbreviated: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: 'મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
                wide: {
                  am: 'AM',
                  pm: 'PM',
                  midnight: '​મધ્યરાત્રિ',
                  noon: 'બપોરે',
                  morning: 'સવારે',
                  afternoon: 'બપોરે',
                  evening: 'સાંજે',
                  night: 'રાત્રે',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
