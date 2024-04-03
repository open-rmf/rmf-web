(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [7393, 69107],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/localize/index.js':
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
                narrow: ['கி.மு.', 'கி.பி.'],
                abbreviated: ['கி.மு.', 'கி.பி.'],
                wide: ['கிறிஸ்துவுக்கு முன்', 'அன்னோ டோமினி'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['காலா.1', 'காலா.2', 'காலா.3', 'காலா.4'],
                wide: [
                  'ஒன்றாம் காலாண்டு',
                  'இரண்டாம் காலாண்டு',
                  'மூன்றாம் காலாண்டு',
                  'நான்காம் காலாண்டு',
                ],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['ஜ', 'பி', 'மா', 'ஏ', 'மே', 'ஜூ', 'ஜூ', 'ஆ', 'செ', 'அ', 'ந', 'டி'],
                abbreviated: [
                  'ஜன.',
                  'பிப்.',
                  'மார்.',
                  'ஏப்.',
                  'மே',
                  'ஜூன்',
                  'ஜூலை',
                  'ஆக.',
                  'செப்.',
                  'அக்.',
                  'நவ.',
                  'டிச.',
                ],
                wide: [
                  'ஜனவரி',
                  'பிப்ரவரி',
                  'மார்ச்',
                  'ஏப்ரல்',
                  'மே',
                  'ஜூன்',
                  'ஜூலை',
                  'ஆகஸ்ட்',
                  'செப்டம்பர்',
                  'அக்டோபர்',
                  'நவம்பர்',
                  'டிசம்பர்',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ஞா', 'தி', 'செ', 'பு', 'வி', 'வெ', 'ச'],
                short: ['ஞா', 'தி', 'செ', 'பு', 'வி', 'வெ', 'ச'],
                abbreviated: ['ஞாயி.', 'திங்.', 'செவ்.', 'புத.', 'வியா.', 'வெள்.', 'சனி'],
                wide: ['ஞாயிறு', 'திங்கள்', 'செவ்வாய்', 'புதன்', 'வியாழன்', 'வெள்ளி', 'சனி'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'மு.ப',
                  pm: 'பி.ப',
                  midnight: 'நள்.',
                  noon: 'நண்.',
                  morning: 'கா.',
                  afternoon: 'மதி.',
                  evening: 'மா.',
                  night: 'இர.',
                },
                abbreviated: {
                  am: 'முற்பகல்',
                  pm: 'பிற்பகல்',
                  midnight: 'நள்ளிரவு',
                  noon: 'நண்பகல்',
                  morning: 'காலை',
                  afternoon: 'மதியம்',
                  evening: 'மாலை',
                  night: 'இரவு',
                },
                wide: {
                  am: 'முற்பகல்',
                  pm: 'பிற்பகல்',
                  midnight: 'நள்ளிரவு',
                  noon: 'நண்பகல்',
                  morning: 'காலை',
                  afternoon: 'மதியம்',
                  evening: 'மாலை',
                  night: 'இரவு',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'மு.ப',
                  pm: 'பி.ப',
                  midnight: 'நள்.',
                  noon: 'நண்.',
                  morning: 'கா.',
                  afternoon: 'மதி.',
                  evening: 'மா.',
                  night: 'இர.',
                },
                abbreviated: {
                  am: 'முற்பகல்',
                  pm: 'பிற்பகல்',
                  midnight: 'நள்ளிரவு',
                  noon: 'நண்பகல்',
                  morning: 'காலை',
                  afternoon: 'மதியம்',
                  evening: 'மாலை',
                  night: 'இரவு',
                },
                wide: {
                  am: 'முற்பகல்',
                  pm: 'பிற்பகல்',
                  midnight: 'நள்ளிரவு',
                  noon: 'நண்பகல்',
                  morning: 'காலை',
                  afternoon: 'மதியம்',
                  evening: 'மாலை',
                  night: 'இரவு',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
