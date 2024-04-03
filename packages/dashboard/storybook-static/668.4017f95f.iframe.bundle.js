(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [668, 45585, 69107, 95565, 79489, 24746, 9707, 81417, 7393, 2527],
  {
    '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js':
      (module) => {
        (module.exports = function _interopRequireDefault(obj) {
          return obj && obj.__esModule ? obj : { default: obj };
        }),
          (module.exports.__esModule = !0),
          (module.exports.default = module.exports);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildFormatLongFn/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = function buildFormatLongFn(args) {
            return function () {
              var options = arguments.length > 0 && void 0 !== arguments[0] ? arguments[0] : {},
                width = options.width ? String(options.width) : args.defaultWidth;
              return args.formats[width] || args.formats[args.defaultWidth];
            };
          }),
          (module.exports = exports.default);
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchFn/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = function buildMatchFn(args) {
            return function (string) {
              var options = arguments.length > 1 && void 0 !== arguments[1] ? arguments[1] : {},
                width = options.width,
                matchPattern =
                  (width && args.matchPatterns[width]) ||
                  args.matchPatterns[args.defaultMatchWidth],
                matchResult = string.match(matchPattern);
              if (!matchResult) return null;
              var value,
                matchedString = matchResult[0],
                parsePatterns =
                  (width && args.parsePatterns[width]) ||
                  args.parsePatterns[args.defaultParseWidth],
                key = Array.isArray(parsePatterns)
                  ? (function findIndex(array, predicate) {
                      for (var key = 0; key < array.length; key++)
                        if (predicate(array[key])) return key;
                      return;
                    })(parsePatterns, function (pattern) {
                      return pattern.test(matchedString);
                    })
                  : (function findKey(object, predicate) {
                      for (var key in object)
                        if (object.hasOwnProperty(key) && predicate(object[key])) return key;
                      return;
                    })(parsePatterns, function (pattern) {
                      return pattern.test(matchedString);
                    });
              return (
                (value = args.valueCallback ? args.valueCallback(key) : key),
                {
                  value: (value = options.valueCallback ? options.valueCallback(value) : value),
                  rest: string.slice(matchedString.length),
                }
              );
            };
          }),
          (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchPatternFn/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }),
          (exports.default = function buildMatchPatternFn(args) {
            return function (string) {
              var options = arguments.length > 1 && void 0 !== arguments[1] ? arguments[1] : {},
                matchResult = string.match(args.matchPattern);
              if (!matchResult) return null;
              var matchedString = matchResult[0],
                parseResult = string.match(args.parsePattern);
              if (!parseResult) return null;
              var value = args.valueCallback ? args.valueCallback(parseResult[0]) : parseResult[0];
              return {
                value: (value = options.valueCallback ? options.valueCallback(value) : value),
                rest: string.slice(matchedString.length),
              };
            };
          }),
          (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: {
                default: 'ஒரு வினாடிக்கு குறைவாக',
                in: 'ஒரு வினாடிக்குள்',
                ago: 'ஒரு வினாடிக்கு முன்பு',
              },
              other: {
                default: '{{count}} வினாடிகளுக்கு குறைவாக',
                in: '{{count}} வினாடிகளுக்குள்',
                ago: '{{count}} வினாடிகளுக்கு முன்பு',
              },
            },
            xSeconds: {
              one: { default: '1 வினாடி', in: '1 வினாடியில்', ago: '1 வினாடி முன்பு' },
              other: {
                default: '{{count}} விநாடிகள்',
                in: '{{count}} வினாடிகளில்',
                ago: '{{count}} விநாடிகளுக்கு முன்பு',
              },
            },
            halfAMinute: {
              default: 'அரை நிமிடம்',
              in: 'அரை நிமிடத்தில்',
              ago: 'அரை நிமிடம் முன்பு',
            },
            lessThanXMinutes: {
              one: {
                default: 'ஒரு நிமிடத்திற்கும் குறைவாக',
                in: 'ஒரு நிமிடத்திற்குள்',
                ago: 'ஒரு நிமிடத்திற்கு முன்பு',
              },
              other: {
                default: '{{count}} நிமிடங்களுக்கும் குறைவாக',
                in: '{{count}} நிமிடங்களுக்குள்',
                ago: '{{count}} நிமிடங்களுக்கு முன்பு',
              },
            },
            xMinutes: {
              one: { default: '1 நிமிடம்', in: '1 நிமிடத்தில்', ago: '1 நிமிடம் முன்பு' },
              other: {
                default: '{{count}} நிமிடங்கள்',
                in: '{{count}} நிமிடங்களில்',
                ago: '{{count}} நிமிடங்களுக்கு முன்பு',
              },
            },
            aboutXHours: {
              one: {
                default: 'சுமார் 1 மணி நேரம்',
                in: 'சுமார் 1 மணி நேரத்தில்',
                ago: 'சுமார் 1 மணி நேரத்திற்கு முன்பு',
              },
              other: {
                default: 'சுமார் {{count}} மணி நேரம்',
                in: 'சுமார் {{count}} மணி நேரத்திற்கு முன்பு',
                ago: 'சுமார் {{count}} மணி நேரத்தில்',
              },
            },
            xHours: {
              one: {
                default: '1 மணி நேரம்',
                in: '1 மணி நேரத்தில்',
                ago: '1 மணி நேரத்திற்கு முன்பு',
              },
              other: {
                default: '{{count}} மணி நேரம்',
                in: '{{count}} மணி நேரத்தில்',
                ago: '{{count}} மணி நேரத்திற்கு முன்பு',
              },
            },
            xDays: {
              one: { default: '1 நாள்', in: '1 நாளில்', ago: '1 நாள் முன்பு' },
              other: {
                default: '{{count}} நாட்கள்',
                in: '{{count}} நாட்களில்',
                ago: '{{count}} நாட்களுக்கு முன்பு',
              },
            },
            aboutXWeeks: {
              one: {
                default: 'சுமார் 1 வாரம்',
                in: 'சுமார் 1 வாரத்தில்',
                ago: 'சுமார் 1 வாரம் முன்பு',
              },
              other: {
                default: 'சுமார் {{count}} வாரங்கள்',
                in: 'சுமார் {{count}} வாரங்களில்',
                ago: 'சுமார் {{count}} வாரங்களுக்கு முன்பு',
              },
            },
            xWeeks: {
              one: { default: '1 வாரம்', in: '1 வாரத்தில்', ago: '1 வாரம் முன்பு' },
              other: {
                default: '{{count}} வாரங்கள்',
                in: '{{count}} வாரங்களில்',
                ago: '{{count}} வாரங்களுக்கு முன்பு',
              },
            },
            aboutXMonths: {
              one: {
                default: 'சுமார் 1 மாதம்',
                in: 'சுமார் 1 மாதத்தில்',
                ago: 'சுமார் 1 மாதத்திற்கு முன்பு',
              },
              other: {
                default: 'சுமார் {{count}} மாதங்கள்',
                in: 'சுமார் {{count}} மாதங்களில்',
                ago: 'சுமார் {{count}} மாதங்களுக்கு முன்பு',
              },
            },
            xMonths: {
              one: { default: '1 மாதம்', in: '1 மாதத்தில்', ago: '1 மாதம் முன்பு' },
              other: {
                default: '{{count}} மாதங்கள்',
                in: '{{count}} மாதங்களில்',
                ago: '{{count}} மாதங்களுக்கு முன்பு',
              },
            },
            aboutXYears: {
              one: {
                default: 'சுமார் 1 வருடம்',
                in: 'சுமார் 1 ஆண்டில்',
                ago: 'சுமார் 1 வருடம் முன்பு',
              },
              other: {
                default: 'சுமார் {{count}} ஆண்டுகள்',
                in: 'சுமார் {{count}} ஆண்டுகளில்',
                ago: 'சுமார் {{count}} ஆண்டுகளுக்கு முன்பு',
              },
            },
            xYears: {
              one: { default: '1 வருடம்', in: '1 ஆண்டில்', ago: '1 வருடம் முன்பு' },
              other: {
                default: '{{count}} ஆண்டுகள்',
                in: '{{count}} ஆண்டுகளில்',
                ago: '{{count}} ஆண்டுகளுக்கு முன்பு',
              },
            },
            overXYears: {
              one: {
                default: '1 வருடத்திற்கு மேல்',
                in: '1 வருடத்திற்கும் மேலாக',
                ago: '1 வருடம் முன்பு',
              },
              other: {
                default: '{{count}} ஆண்டுகளுக்கும் மேலாக',
                in: '{{count}} ஆண்டுகளில்',
                ago: '{{count}} ஆண்டுகளுக்கு முன்பு',
              },
            },
            almostXYears: {
              one: {
                default: 'கிட்டத்தட்ட 1 வருடம்',
                in: 'கிட்டத்தட்ட 1 ஆண்டில்',
                ago: 'கிட்டத்தட்ட 1 வருடம் முன்பு',
              },
              other: {
                default: 'கிட்டத்தட்ட {{count}} ஆண்டுகள்',
                in: 'கிட்டத்தட்ட {{count}} ஆண்டுகளில்',
                ago: 'கிட்டத்தட்ட {{count}} ஆண்டுகளுக்கு முன்பு',
              },
            },
          },
          _default = function formatDistance(token, count, options) {
            var tense =
                null != options && options.addSuffix
                  ? options.comparison && options.comparison > 0
                    ? 'in'
                    : 'ago'
                  : 'default',
              tokenValue = formatDistanceLocale[token];
            return (function isPluralType(val) {
              return void 0 !== val.one;
            })(tokenValue)
              ? 1 === count
                ? tokenValue.one[tense]
                : tokenValue.other[tense].replace('{{count}}', String(count))
              : tokenValue[tense];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatLong/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildFormatLongFn/index.js',
            ),
          ),
          _default = {
            date: (0, _index.default)({
              formats: {
                full: 'EEEE, d MMMM, y',
                long: 'd MMMM, y',
                medium: 'd MMM, y',
                short: 'd/M/yy',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'a h:mm:ss zzzz',
                long: 'a h:mm:ss z',
                medium: 'a h:mm:ss',
                short: 'a h:mm',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: '{{date}} {{time}}',
                long: '{{date}} {{time}}',
                medium: '{{date}}, {{time}}',
                short: '{{date}}, {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'கடந்த' eeee p 'மணிக்கு'",
            yesterday: "'நேற்று ' p 'மணிக்கு'",
            today: "'இன்று ' p 'மணிக்கு'",
            tomorrow: "'நாளை ' p 'மணிக்கு'",
            nextWeek: "eeee p 'மணிக்கு'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/match/index.js':
      (module, exports, __webpack_require__) => {
        'use strict';
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchFn/index.js',
            ),
          ),
          _default = {
            ordinalNumber: (0,
            _interopRequireDefault(
              __webpack_require__(
                '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/_lib/buildMatchPatternFn/index.js',
              ),
            ).default)({
              matchPattern: /^(\d+)(வது)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(கி.மு.|கி.பி.)/i,
                abbreviated: /^(கி\.?\s?மு\.?|கி\.?\s?பி\.?)/,
                wide: /^(கிறிஸ்துவுக்கு\sமுன்|அன்னோ\sடோமினி)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/கி\.?\s?மு\.?/, /கி\.?\s?பி\.?/] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^காலா.[1234]/i,
                wide: /^(ஒன்றாம்|இரண்டாம்|மூன்றாம்|நான்காம்) காலாண்டு/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/1/i, /2/i, /3/i, /4/i],
                any: [
                  /(1|காலா.1|ஒன்றாம்)/i,
                  /(2|காலா.2|இரண்டாம்)/i,
                  /(3|காலா.3|மூன்றாம்)/i,
                  /(4|காலா.4|நான்காம்)/i,
                ],
              },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ஜ|பி|மா|ஏ|மே|ஜூ|ஆ|செ|அ|ந|டி)$/i,
                abbreviated: /^(ஜன.|பிப்.|மார்.|ஏப்.|மே|ஜூன்|ஜூலை|ஆக.|செப்.|அக்.|நவ.|டிச.)/i,
                wide: /^(ஜனவரி|பிப்ரவரி|மார்ச்|ஏப்ரல்|மே|ஜூன்|ஜூலை|ஆகஸ்ட்|செப்டம்பர்|அக்டோபர்|நவம்பர்|டிசம்பர்)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^ஜ$/i,
                  /^பி/i,
                  /^மா/i,
                  /^ஏ/i,
                  /^மே/i,
                  /^ஜூ/i,
                  /^ஜூ/i,
                  /^ஆ/i,
                  /^செ/i,
                  /^அ/i,
                  /^ந/i,
                  /^டி/i,
                ],
                any: [
                  /^ஜன/i,
                  /^பி/i,
                  /^மா/i,
                  /^ஏ/i,
                  /^மே/i,
                  /^ஜூன்/i,
                  /^ஜூலை/i,
                  /^ஆ/i,
                  /^செ/i,
                  /^அ/i,
                  /^ந/i,
                  /^டி/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ஞா|தி|செ|பு|வி|வெ|ச)/i,
                short: /^(ஞா|தி|செ|பு|வி|வெ|ச)/i,
                abbreviated: /^(ஞாயி.|திங்.|செவ்.|புத.|வியா.|வெள்.|சனி)/i,
                wide: /^(ஞாயிறு|திங்கள்|செவ்வாய்|புதன்|வியாழன்|வெள்ளி|சனி)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^ஞா/i, /^தி/i, /^செ/i, /^பு/i, /^வி/i, /^வெ/i, /^ச/i],
                any: [/^ஞா/i, /^தி/i, /^செ/i, /^பு/i, /^வி/i, /^வெ/i, /^ச/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(மு.ப|பி.ப|நள்|நண்|காலை|மதியம்|மாலை|இரவு)/i,
                any: /^(மு.ப|பி.ப|முற்பகல்|பிற்பகல்|நள்ளிரவு|நண்பகல்|காலை|மதியம்|மாலை|இரவு)/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^மு/i,
                  pm: /^பி/i,
                  midnight: /^நள்/i,
                  noon: /^நண்/i,
                  morning: /காலை/i,
                  afternoon: /மதியம்/i,
                  evening: /மாலை/i,
                  night: /இரவு/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/index.js': (
      module,
      exports,
      __webpack_require__,
    ) => {
      'use strict';
      var _interopRequireDefault = __webpack_require__(
        '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
      ).default;
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
      var _index = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'ta',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 1, firstWeekContainsDate: 4 },
        };
      (exports.default = _default), (module.exports = exports.default);
    },
  },
]);
