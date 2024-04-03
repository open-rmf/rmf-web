(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [70142, 45585, 69107, 95565, 79489, 28020, 74181, 12583, 50023, 40425],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
          lessThanXSeconds: {
            one: {
              default: '1 ಸೆಕೆಂಡ್‌ಗಿಂತ ಕಡಿಮೆ',
              future: '1 ಸೆಕೆಂಡ್‌ಗಿಂತ ಕಡಿಮೆ',
              past: '1 ಸೆಕೆಂಡ್‌ಗಿಂತ ಕಡಿಮೆ',
            },
            other: {
              default: '{{count}} ಸೆಕೆಂಡ್‌ಗಿಂತ ಕಡಿಮೆ',
              future: '{{count}} ಸೆಕೆಂಡ್‌ಗಿಂತ ಕಡಿಮೆ',
              past: '{{count}} ಸೆಕೆಂಡ್‌ಗಿಂತ ಕಡಿಮೆ',
            },
          },
          xSeconds: {
            one: { default: '1 ಸೆಕೆಂಡ್', future: '1 ಸೆಕೆಂಡ್‌ನಲ್ಲಿ', past: '1 ಸೆಕೆಂಡ್ ಹಿಂದೆ' },
            other: {
              default: '{{count}} ಸೆಕೆಂಡುಗಳು',
              future: '{{count}} ಸೆಕೆಂಡ್‌ಗಳಲ್ಲಿ',
              past: '{{count}} ಸೆಕೆಂಡ್ ಹಿಂದೆ',
            },
          },
          halfAMinute: {
            other: { default: 'ಅರ್ಧ ನಿಮಿಷ', future: 'ಅರ್ಧ ನಿಮಿಷದಲ್ಲಿ', past: 'ಅರ್ಧ ನಿಮಿಷದ ಹಿಂದೆ' },
          },
          lessThanXMinutes: {
            one: {
              default: '1 ನಿಮಿಷಕ್ಕಿಂತ ಕಡಿಮೆ',
              future: '1 ನಿಮಿಷಕ್ಕಿಂತ ಕಡಿಮೆ',
              past: '1 ನಿಮಿಷಕ್ಕಿಂತ ಕಡಿಮೆ',
            },
            other: {
              default: '{{count}} ನಿಮಿಷಕ್ಕಿಂತ ಕಡಿಮೆ',
              future: '{{count}} ನಿಮಿಷಕ್ಕಿಂತ ಕಡಿಮೆ',
              past: '{{count}} ನಿಮಿಷಕ್ಕಿಂತ ಕಡಿಮೆ',
            },
          },
          xMinutes: {
            one: { default: '1 ನಿಮಿಷ', future: '1 ನಿಮಿಷದಲ್ಲಿ', past: '1 ನಿಮಿಷದ ಹಿಂದೆ' },
            other: {
              default: '{{count}} ನಿಮಿಷಗಳು',
              future: '{{count}} ನಿಮಿಷಗಳಲ್ಲಿ',
              past: '{{count}} ನಿಮಿಷಗಳ ಹಿಂದೆ',
            },
          },
          aboutXHours: {
            one: {
              default: 'ಸುಮಾರು 1 ಗಂಟೆ',
              future: 'ಸುಮಾರು 1 ಗಂಟೆಯಲ್ಲಿ',
              past: 'ಸುಮಾರು 1 ಗಂಟೆ ಹಿಂದೆ',
            },
            other: {
              default: 'ಸುಮಾರು {{count}} ಗಂಟೆಗಳು',
              future: 'ಸುಮಾರು {{count}} ಗಂಟೆಗಳಲ್ಲಿ',
              past: 'ಸುಮಾರು {{count}} ಗಂಟೆಗಳ ಹಿಂದೆ',
            },
          },
          xHours: {
            one: { default: '1 ಗಂಟೆ', future: '1 ಗಂಟೆಯಲ್ಲಿ', past: '1 ಗಂಟೆ ಹಿಂದೆ' },
            other: {
              default: '{{count}} ಗಂಟೆಗಳು',
              future: '{{count}} ಗಂಟೆಗಳಲ್ಲಿ',
              past: '{{count}} ಗಂಟೆಗಳ ಹಿಂದೆ',
            },
          },
          xDays: {
            one: { default: '1 ದಿನ', future: '1 ದಿನದಲ್ಲಿ', past: '1 ದಿನದ ಹಿಂದೆ' },
            other: {
              default: '{{count}} ದಿನಗಳು',
              future: '{{count}} ದಿನಗಳಲ್ಲಿ',
              past: '{{count}} ದಿನಗಳ ಹಿಂದೆ',
            },
          },
          aboutXMonths: {
            one: {
              default: 'ಸುಮಾರು 1 ತಿಂಗಳು',
              future: 'ಸುಮಾರು 1 ತಿಂಗಳಲ್ಲಿ',
              past: 'ಸುಮಾರು 1 ತಿಂಗಳ ಹಿಂದೆ',
            },
            other: {
              default: 'ಸುಮಾರು {{count}} ತಿಂಗಳು',
              future: 'ಸುಮಾರು {{count}} ತಿಂಗಳುಗಳಲ್ಲಿ',
              past: 'ಸುಮಾರು {{count}} ತಿಂಗಳುಗಳ ಹಿಂದೆ',
            },
          },
          xMonths: {
            one: { default: '1 ತಿಂಗಳು', future: '1 ತಿಂಗಳಲ್ಲಿ', past: '1 ತಿಂಗಳ ಹಿಂದೆ' },
            other: {
              default: '{{count}} ತಿಂಗಳು',
              future: '{{count}} ತಿಂಗಳುಗಳಲ್ಲಿ',
              past: '{{count}} ತಿಂಗಳುಗಳ ಹಿಂದೆ',
            },
          },
          aboutXYears: {
            one: {
              default: 'ಸುಮಾರು 1 ವರ್ಷ',
              future: 'ಸುಮಾರು 1 ವರ್ಷದಲ್ಲಿ',
              past: 'ಸುಮಾರು 1 ವರ್ಷದ ಹಿಂದೆ',
            },
            other: {
              default: 'ಸುಮಾರು {{count}} ವರ್ಷಗಳು',
              future: 'ಸುಮಾರು {{count}} ವರ್ಷಗಳಲ್ಲಿ',
              past: 'ಸುಮಾರು {{count}} ವರ್ಷಗಳ ಹಿಂದೆ',
            },
          },
          xYears: {
            one: { default: '1 ವರ್ಷ', future: '1 ವರ್ಷದಲ್ಲಿ', past: '1 ವರ್ಷದ ಹಿಂದೆ' },
            other: {
              default: '{{count}} ವರ್ಷಗಳು',
              future: '{{count}} ವರ್ಷಗಳಲ್ಲಿ',
              past: '{{count}} ವರ್ಷಗಳ ಹಿಂದೆ',
            },
          },
          overXYears: {
            one: { default: '1 ವರ್ಷದ ಮೇಲೆ', future: '1 ವರ್ಷದ ಮೇಲೆ', past: '1 ವರ್ಷದ ಮೇಲೆ' },
            other: {
              default: '{{count}} ವರ್ಷಗಳ ಮೇಲೆ',
              future: '{{count}} ವರ್ಷಗಳ ಮೇಲೆ',
              past: '{{count}} ವರ್ಷಗಳ ಮೇಲೆ',
            },
          },
          almostXYears: {
            one: {
              default: 'ಬಹುತೇಕ 1 ವರ್ಷದಲ್ಲಿ',
              future: 'ಬಹುತೇಕ 1 ವರ್ಷದಲ್ಲಿ',
              past: 'ಬಹುತೇಕ 1 ವರ್ಷದಲ್ಲಿ',
            },
            other: {
              default: 'ಬಹುತೇಕ {{count}} ವರ್ಷಗಳಲ್ಲಿ',
              future: 'ಬಹುತೇಕ {{count}} ವರ್ಷಗಳಲ್ಲಿ',
              past: 'ಬಹುತೇಕ {{count}} ವರ್ಷಗಳಲ್ಲಿ',
            },
          },
        };
        function getResultByTense(parentToken, options) {
          return null != options && options.addSuffix
            ? options.comparison && options.comparison > 0
              ? parentToken.future
              : parentToken.past
            : parentToken.default;
        }
        var _default = function formatDistance(token, count, options) {
          var tokenValue = formatDistanceLocale[token];
          return (
            tokenValue.one && 1 === count
              ? getResultByTense(tokenValue.one, options)
              : getResultByTense(tokenValue.other, options)
          ).replace('{{count}}', String(count));
        };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatLong/index.js':
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
                full: 'EEEE, MMMM d, y',
                long: 'MMMM d, y',
                medium: 'MMM d, y',
                short: 'd/M/yy',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'hh:mm:ss a zzzz',
                long: 'hh:mm:ss a z',
                medium: 'hh:mm:ss a',
                short: 'hh:mm a',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: '{{date}} {{time}}',
                long: '{{date}} {{time}}',
                medium: '{{date}} {{time}}',
                short: '{{date}} {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'ಕಳೆದ' eeee p 'ಕ್ಕೆ'",
            yesterday: "'ನಿನ್ನೆ' p 'ಕ್ಕೆ'",
            today: "'ಇಂದು' p 'ಕ್ಕೆ'",
            tomorrow: "'ನಾಳೆ' p 'ಕ್ಕೆ'",
            nextWeek: "eeee p 'ಕ್ಕೆ'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/localize/index.js':
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
              return Number(dirtyNumber) + 'ನೇ';
            },
            era: (0, _index.default)({
              values: {
                narrow: ['ಕ್ರಿ.ಪೂ', 'ಕ್ರಿ.ಶ'],
                abbreviated: ['ಕ್ರಿ.ಪೂ', 'ಕ್ರಿ.ಶ'],
                wide: ['ಕ್ರಿಸ್ತ ಪೂರ್ವ', 'ಕ್ರಿಸ್ತ ಶಕ'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['ತ್ರೈ 1', 'ತ್ರೈ 2', 'ತ್ರೈ 3', 'ತ್ರೈ 4'],
                wide: ['1ನೇ ತ್ರೈಮಾಸಿಕ', '2ನೇ ತ್ರೈಮಾಸಿಕ', '3ನೇ ತ್ರೈಮಾಸಿಕ', '4ನೇ ತ್ರೈಮಾಸಿಕ'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['ಜ', 'ಫೆ', 'ಮಾ', 'ಏ', 'ಮೇ', 'ಜೂ', 'ಜು', 'ಆ', 'ಸೆ', 'ಅ', 'ನ', 'ಡಿ'],
                abbreviated: [
                  'ಜನ',
                  'ಫೆಬ್ರ',
                  'ಮಾರ್ಚ್',
                  'ಏಪ್ರಿ',
                  'ಮೇ',
                  'ಜೂನ್',
                  'ಜುಲೈ',
                  'ಆಗ',
                  'ಸೆಪ್ಟೆಂ',
                  'ಅಕ್ಟೋ',
                  'ನವೆಂ',
                  'ಡಿಸೆಂ',
                ],
                wide: [
                  'ಜನವರಿ',
                  'ಫೆಬ್ರವರಿ',
                  'ಮಾರ್ಚ್',
                  'ಏಪ್ರಿಲ್',
                  'ಮೇ',
                  'ಜೂನ್',
                  'ಜುಲೈ',
                  'ಆಗಸ್ಟ್',
                  'ಸೆಪ್ಟೆಂಬರ್',
                  'ಅಕ್ಟೋಬರ್',
                  'ನವೆಂಬರ್',
                  'ಡಿಸೆಂಬರ್',
                ],
              },
              defaultWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['ಭಾ', 'ಸೋ', 'ಮಂ', 'ಬು', 'ಗು', 'ಶು', 'ಶ'],
                short: ['ಭಾನು', 'ಸೋಮ', 'ಮಂಗಳ', 'ಬುಧ', 'ಗುರು', 'ಶುಕ್ರ', 'ಶನಿ'],
                abbreviated: ['ಭಾನು', 'ಸೋಮ', 'ಮಂಗಳ', 'ಬುಧ', 'ಗುರು', 'ಶುಕ್ರ', 'ಶನಿ'],
                wide: ['ಭಾನುವಾರ', 'ಸೋಮವಾರ', 'ಮಂಗಳವಾರ', 'ಬುಧವಾರ', 'ಗುರುವಾರ', 'ಶುಕ್ರವಾರ', 'ಶನಿವಾರ'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ಪೂರ್ವಾಹ್ನ',
                  pm: 'ಅಪರಾಹ್ನ',
                  midnight: 'ಮಧ್ಯರಾತ್ರಿ',
                  noon: 'ಮಧ್ಯಾಹ್ನ',
                  morning: 'ಬೆಳಗ್ಗೆ',
                  afternoon: 'ಮಧ್ಯಾಹ್ನ',
                  evening: 'ಸಂಜೆ',
                  night: 'ರಾತ್ರಿ',
                },
                abbreviated: {
                  am: 'ಪೂರ್ವಾಹ್ನ',
                  pm: 'ಅಪರಾಹ್ನ',
                  midnight: 'ಮಧ್ಯರಾತ್ರಿ',
                  noon: 'ಮಧ್ಯಾನ್ಹ',
                  morning: 'ಬೆಳಗ್ಗೆ',
                  afternoon: 'ಮಧ್ಯಾನ್ಹ',
                  evening: 'ಸಂಜೆ',
                  night: 'ರಾತ್ರಿ',
                },
                wide: {
                  am: 'ಪೂರ್ವಾಹ್ನ',
                  pm: 'ಅಪರಾಹ್ನ',
                  midnight: 'ಮಧ್ಯರಾತ್ರಿ',
                  noon: 'ಮಧ್ಯಾನ್ಹ',
                  morning: 'ಬೆಳಗ್ಗೆ',
                  afternoon: 'ಮಧ್ಯಾನ್ಹ',
                  evening: 'ಸಂಜೆ',
                  night: 'ರಾತ್ರಿ',
                },
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: {
                  am: 'ಪೂ',
                  pm: 'ಅ',
                  midnight: 'ಮಧ್ಯರಾತ್ರಿ',
                  noon: 'ಮಧ್ಯಾನ್ಹ',
                  morning: 'ಬೆಳಗ್ಗೆ',
                  afternoon: 'ಮಧ್ಯಾನ್ಹ',
                  evening: 'ಸಂಜೆ',
                  night: 'ರಾತ್ರಿ',
                },
                abbreviated: {
                  am: 'ಪೂರ್ವಾಹ್ನ',
                  pm: 'ಅಪರಾಹ್ನ',
                  midnight: 'ಮಧ್ಯ ರಾತ್ರಿ',
                  noon: 'ಮಧ್ಯಾನ್ಹ',
                  morning: 'ಬೆಳಗ್ಗೆ',
                  afternoon: 'ಮಧ್ಯಾನ್ಹ',
                  evening: 'ಸಂಜೆ',
                  night: 'ರಾತ್ರಿ',
                },
                wide: {
                  am: 'ಪೂರ್ವಾಹ್ನ',
                  pm: 'ಅಪರಾಹ್ನ',
                  midnight: 'ಮಧ್ಯ ರಾತ್ರಿ',
                  noon: 'ಮಧ್ಯಾನ್ಹ',
                  morning: 'ಬೆಳಗ್ಗೆ',
                  afternoon: 'ಮಧ್ಯಾನ್ಹ',
                  evening: 'ಸಂಜೆ',
                  night: 'ರಾತ್ರಿ',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/match/index.js':
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
              matchPattern: /^(\d+)(ನೇ|ನೆ)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ಕ್ರಿ.ಪೂ|ಕ್ರಿ.ಶ)/i,
                abbreviated: /^(ಕ್ರಿ\.?\s?ಪೂ\.?|ಕ್ರಿ\.?\s?ಶ\.?|ಪ್ರ\.?\s?ಶ\.?)/i,
                wide: /^(ಕ್ರಿಸ್ತ ಪೂರ್ವ|ಕ್ರಿಸ್ತ ಶಕ|ಪ್ರಸಕ್ತ ಶಕ)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^ಪೂ/i, /^(ಶ|ಪ್ರ)/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^ತ್ರೈ[1234]|ತ್ರೈ [1234]| [1234]ತ್ರೈ/i,
                wide: /^[1234](ನೇ)? ತ್ರೈಮಾಸಿಕ/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/1/i, /2/i, /3/i, /4/i] },
              defaultParseWidth: 'any',
              valueCallback: function valueCallback(index) {
                return index + 1;
              },
            }),
            month: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ಜೂ|ಜು|ಜ|ಫೆ|ಮಾ|ಏ|ಮೇ|ಆ|ಸೆ|ಅ|ನ|ಡಿ)/i,
                abbreviated: /^(ಜನ|ಫೆಬ್ರ|ಮಾರ್ಚ್|ಏಪ್ರಿ|ಮೇ|ಜೂನ್|ಜುಲೈ|ಆಗ|ಸೆಪ್ಟೆಂ|ಅಕ್ಟೋ|ನವೆಂ|ಡಿಸೆಂ)/i,
                wide: /^(ಜನವರಿ|ಫೆಬ್ರವರಿ|ಮಾರ್ಚ್|ಏಪ್ರಿಲ್|ಮೇ|ಜೂನ್|ಜುಲೈ|ಆಗಸ್ಟ್|ಸೆಪ್ಟೆಂಬರ್|ಅಕ್ಟೋಬರ್|ನವೆಂಬರ್|ಡಿಸೆಂಬರ್)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^ಜ$/i,
                  /^ಫೆ/i,
                  /^ಮಾ/i,
                  /^ಏ/i,
                  /^ಮೇ/i,
                  /^ಜೂ/i,
                  /^ಜು$/i,
                  /^ಆ/i,
                  /^ಸೆ/i,
                  /^ಅ/i,
                  /^ನ/i,
                  /^ಡಿ/i,
                ],
                any: [
                  /^ಜನ/i,
                  /^ಫೆ/i,
                  /^ಮಾ/i,
                  /^ಏ/i,
                  /^ಮೇ/i,
                  /^ಜೂನ್/i,
                  /^ಜುಲೈ/i,
                  /^ಆ/i,
                  /^ಸೆ/i,
                  /^ಅ/i,
                  /^ನ/i,
                  /^ಡಿ/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ಭಾ|ಸೋ|ಮ|ಬು|ಗು|ಶು|ಶ)/i,
                short: /^(ಭಾನು|ಸೋಮ|ಮಂಗಳ|ಬುಧ|ಗುರು|ಶುಕ್ರ|ಶನಿ)/i,
                abbreviated: /^(ಭಾನು|ಸೋಮ|ಮಂಗಳ|ಬುಧ|ಗುರು|ಶುಕ್ರ|ಶನಿ)/i,
                wide: /^(ಭಾನುವಾರ|ಸೋಮವಾರ|ಮಂಗಳವಾರ|ಬುಧವಾರ|ಗುರುವಾರ|ಶುಕ್ರವಾರ|ಶನಿವಾರ)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^ಭಾ/i, /^ಸೋ/i, /^ಮ/i, /^ಬು/i, /^ಗು/i, /^ಶು/i, /^ಶ/i],
                any: [/^ಭಾ/i, /^ಸೋ/i, /^ಮ/i, /^ಬು/i, /^ಗು/i, /^ಶು/i, /^ಶ/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ಪೂ|ಅ|ಮಧ್ಯರಾತ್ರಿ|ಮಧ್ಯಾನ್ಹ|ಬೆಳಗ್ಗೆ|ಸಂಜೆ|ರಾತ್ರಿ)/i,
                any: /^(ಪೂರ್ವಾಹ್ನ|ಅಪರಾಹ್ನ|ಮಧ್ಯರಾತ್ರಿ|ಮಧ್ಯಾನ್ಹ|ಬೆಳಗ್ಗೆ|ಸಂಜೆ|ರಾತ್ರಿ)/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^ಪೂ/i,
                  pm: /^ಅ/i,
                  midnight: /ಮಧ್ಯರಾತ್ರಿ/i,
                  noon: /ಮಧ್ಯಾನ್ಹ/i,
                  morning: /ಬೆಳಗ್ಗೆ/i,
                  afternoon: /ಮಧ್ಯಾನ್ಹ/i,
                  evening: /ಸಂಜೆ/i,
                  night: /ರಾತ್ರಿ/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'kn',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 1, firstWeekContainsDate: 1 },
        };
      (exports.default = _default), (module.exports = exports.default);
    },
  },
]);
