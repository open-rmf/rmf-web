(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [61781, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/localize/index.js':
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
              return Number(dirtyNumber) + 'వ';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['క్రీ.పూ.', 'క్రీ.శ.'],
                abbreviated: ['క్రీ.పూ.', 'క్రీ.శ.'],
                wide: ['క్రీస్తు పూర్వం', 'క్రీస్తుశకం'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['త్రై1', 'త్రై2', 'త్రై3', 'త్రై4'],
                wide: ['1వ త్రైమాసికం', '2వ త్రైమాసికం', '3వ త్రైమాసికం', '4వ త్రైమాసికం'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['జ', 'ఫి', 'మా', 'ఏ', 'మే', 'జూ', 'జు', 'ఆ', 'సె', 'అ', 'న', 'డి'],
                abbreviated: [
                  'జన',
                  'ఫిబ్ర',
                  'మార్చి',
                  'ఏప్రి',
                  'మే',
                  'జూన్',
                  'జులై',
                  'ఆగ',
                  'సెప్టెం',
                  'అక్టో',
                  'నవం',
                  'డిసెం',
                ],
                wide: [
                  'జనవరి',
                  'ఫిబ్రవరి',
                  'మార్చి',
                  'ఏప్రిల్',
                  'మే',
                  'జూన్',
                  'జులై',
                  'ఆగస్టు',
                  'సెప్టెంబర్',
                  'అక్టోబర్',
                  'నవంబర్',
                  'డిసెంబర్',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ఆ', 'సో', 'మ', 'బు', 'గు', 'శు', 'శ'],
                short: ['ఆది', 'సోమ', 'మంగళ', 'బుధ', 'గురు', 'శుక్ర', 'శని'],
                abbreviated: ['ఆది', 'సోమ', 'మంగళ', 'బుధ', 'గురు', 'శుక్ర', 'శని'],
                wide: [
                  'ఆదివారం',
                  'సోమవారం',
                  'మంగళవారం',
                  'బుధవారం',
                  'గురువారం',
                  'శుక్రవారం',
                  'శనివారం',
                ],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'పూర్వాహ్నం',
                  pm: 'అపరాహ్నం',
                  midnight: 'అర్ధరాత్రి',
                  noon: 'మిట్టమధ్యాహ్నం',
                  morning: 'ఉదయం',
                  afternoon: 'మధ్యాహ్నం',
                  evening: 'సాయంత్రం',
                  night: 'రాత్రి',
                },
                abbreviated: {
                  am: 'పూర్వాహ్నం',
                  pm: 'అపరాహ్నం',
                  midnight: 'అర్ధరాత్రి',
                  noon: 'మిట్టమధ్యాహ్నం',
                  morning: 'ఉదయం',
                  afternoon: 'మధ్యాహ్నం',
                  evening: 'సాయంత్రం',
                  night: 'రాత్రి',
                },
                wide: {
                  am: 'పూర్వాహ్నం',
                  pm: 'అపరాహ్నం',
                  midnight: 'అర్ధరాత్రి',
                  noon: 'మిట్టమధ్యాహ్నం',
                  morning: 'ఉదయం',
                  afternoon: 'మధ్యాహ్నం',
                  evening: 'సాయంత్రం',
                  night: 'రాత్రి',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'పూర్వాహ్నం',
                  pm: 'అపరాహ్నం',
                  midnight: 'అర్ధరాత్రి',
                  noon: 'మిట్టమధ్యాహ్నం',
                  morning: 'ఉదయం',
                  afternoon: 'మధ్యాహ్నం',
                  evening: 'సాయంత్రం',
                  night: 'రాత్రి',
                },
                abbreviated: {
                  am: 'పూర్వాహ్నం',
                  pm: 'అపరాహ్నం',
                  midnight: 'అర్ధరాత్రి',
                  noon: 'మిట్టమధ్యాహ్నం',
                  morning: 'ఉదయం',
                  afternoon: 'మధ్యాహ్నం',
                  evening: 'సాయంత్రం',
                  night: 'రాత్రి',
                },
                wide: {
                  am: 'పూర్వాహ్నం',
                  pm: 'అపరాహ్నం',
                  midnight: 'అర్ధరాత్రి',
                  noon: 'మిట్టమధ్యాహ్నం',
                  morning: 'ఉదయం',
                  afternoon: 'మధ్యాహ్నం',
                  evening: 'సాయంత్రం',
                  night: 'రాత్రి',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
