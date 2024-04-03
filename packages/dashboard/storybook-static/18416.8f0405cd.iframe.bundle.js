(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [18416, 45585, 69107, 95565, 79489, 47902, 41743, 75093, 61781, 78971],
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatDistance/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              standalone: { one: 'సెకను కన్నా తక్కువ', other: '{{count}} సెకన్ల కన్నా తక్కువ' },
              withPreposition: { one: 'సెకను', other: '{{count}} సెకన్ల' },
            },
            xSeconds: {
              standalone: { one: 'ఒక సెకను', other: '{{count}} సెకన్ల' },
              withPreposition: { one: 'ఒక సెకను', other: '{{count}} సెకన్ల' },
            },
            halfAMinute: { standalone: 'అర నిమిషం', withPreposition: 'అర నిమిషం' },
            lessThanXMinutes: {
              standalone: {
                one: 'ఒక నిమిషం కన్నా తక్కువ',
                other: '{{count}} నిమిషాల కన్నా తక్కువ',
              },
              withPreposition: { one: 'ఒక నిమిషం', other: '{{count}} నిమిషాల' },
            },
            xMinutes: {
              standalone: { one: 'ఒక నిమిషం', other: '{{count}} నిమిషాలు' },
              withPreposition: { one: 'ఒక నిమిషం', other: '{{count}} నిమిషాల' },
            },
            aboutXHours: {
              standalone: { one: 'సుమారు ఒక గంట', other: 'సుమారు {{count}} గంటలు' },
              withPreposition: { one: 'సుమారు ఒక గంట', other: 'సుమారు {{count}} గంటల' },
            },
            xHours: {
              standalone: { one: 'ఒక గంట', other: '{{count}} గంటలు' },
              withPreposition: { one: 'ఒక గంట', other: '{{count}} గంటల' },
            },
            xDays: {
              standalone: { one: 'ఒక రోజు', other: '{{count}} రోజులు' },
              withPreposition: { one: 'ఒక రోజు', other: '{{count}} రోజుల' },
            },
            aboutXWeeks: {
              standalone: { one: 'సుమారు ఒక వారం', other: 'సుమారు {{count}} వారాలు' },
              withPreposition: { one: 'సుమారు ఒక వారం', other: 'సుమారు {{count}} వారాలల' },
            },
            xWeeks: {
              standalone: { one: 'ఒక వారం', other: '{{count}} వారాలు' },
              withPreposition: { one: 'ఒక వారం', other: '{{count}} వారాలల' },
            },
            aboutXMonths: {
              standalone: { one: 'సుమారు ఒక నెల', other: 'సుమారు {{count}} నెలలు' },
              withPreposition: { one: 'సుమారు ఒక నెల', other: 'సుమారు {{count}} నెలల' },
            },
            xMonths: {
              standalone: { one: 'ఒక నెల', other: '{{count}} నెలలు' },
              withPreposition: { one: 'ఒక నెల', other: '{{count}} నెలల' },
            },
            aboutXYears: {
              standalone: { one: 'సుమారు ఒక సంవత్సరం', other: 'సుమారు {{count}} సంవత్సరాలు' },
              withPreposition: { one: 'సుమారు ఒక సంవత్సరం', other: 'సుమారు {{count}} సంవత్సరాల' },
            },
            xYears: {
              standalone: { one: 'ఒక సంవత్సరం', other: '{{count}} సంవత్సరాలు' },
              withPreposition: { one: 'ఒక సంవత్సరం', other: '{{count}} సంవత్సరాల' },
            },
            overXYears: {
              standalone: { one: 'ఒక సంవత్సరం పైగా', other: '{{count}} సంవత్సరాలకు పైగా' },
              withPreposition: { one: 'ఒక సంవత్సరం', other: '{{count}} సంవత్సరాల' },
            },
            almostXYears: {
              standalone: { one: 'దాదాపు ఒక సంవత్సరం', other: 'దాదాపు {{count}} సంవత్సరాలు' },
              withPreposition: { one: 'దాదాపు ఒక సంవత్సరం', other: 'దాదాపు {{count}} సంవత్సరాల' },
            },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue =
                null != options && options.addSuffix
                  ? formatDistanceLocale[token].withPreposition
                  : formatDistanceLocale[token].standalone;
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? result + 'లో'
                  : result + ' క్రితం'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatLong/index.js':
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
                full: 'd, MMMM y, EEEE',
                long: 'd MMMM, y',
                medium: 'd MMM, y',
                short: 'dd-MM-yy',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'h:mm:ss a zzzz',
                long: 'h:mm:ss a z',
                medium: 'h:mm:ss a',
                short: 'h:mm a',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: {
                full: "{{date}} {{time}}'కి'",
                long: "{{date}} {{time}}'కి'",
                medium: '{{date}} {{time}}',
                short: '{{date}} {{time}}',
              },
              defaultWidth: 'full',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatRelative/index.js':
      (module, exports) => {
        'use strict';
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'గత' eeee p",
            yesterday: "'నిన్న' p",
            today: "'ఈ రోజు' p",
            tomorrow: "'రేపు' p",
            nextWeek: "'తదుపరి' eeee p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
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
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/match/index.js':
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
              matchPattern: /^(\d+)(వ)?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^(క్రీ\.పూ\.|క్రీ\.శ\.)/i,
                abbreviated:
                  /^(క్రీ\.?\s?పూ\.?|ప్ర\.?\s?శ\.?\s?పూ\.?|క్రీ\.?\s?శ\.?|సా\.?\s?శ\.?)/i,
                wide: /^(క్రీస్తు పూర్వం|ప్రస్తుత శకానికి పూర్వం|క్రీస్తు శకం|ప్రస్తుత శకం)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^(పూ|శ)/i, /^సా/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^త్రై[1234]/i,
                wide: /^[1234](వ)? త్రైమాసికం/i,
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
                narrow: /^(జూ|జు|జ|ఫి|మా|ఏ|మే|ఆ|సె|అ|న|డి)/i,
                abbreviated: /^(జన|ఫిబ్ర|మార్చి|ఏప్రి|మే|జూన్|జులై|ఆగ|సెప్|అక్టో|నవ|డిసె)/i,
                wide: /^(జనవరి|ఫిబ్రవరి|మార్చి|ఏప్రిల్|మే|జూన్|జులై|ఆగస్టు|సెప్టెంబర్|అక్టోబర్|నవంబర్|డిసెంబర్)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^జ/i,
                  /^ఫి/i,
                  /^మా/i,
                  /^ఏ/i,
                  /^మే/i,
                  /^జూ/i,
                  /^జు/i,
                  /^ఆ/i,
                  /^సె/i,
                  /^అ/i,
                  /^న/i,
                  /^డి/i,
                ],
                any: [
                  /^జన/i,
                  /^ఫి/i,
                  /^మా/i,
                  /^ఏ/i,
                  /^మే/i,
                  /^జూన్/i,
                  /^జులై/i,
                  /^ఆగ/i,
                  /^సె/i,
                  /^అ/i,
                  /^న/i,
                  /^డి/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^(ఆ|సో|మ|బు|గు|శు|శ)/i,
                short: /^(ఆది|సోమ|మం|బుధ|గురు|శుక్ర|శని)/i,
                abbreviated: /^(ఆది|సోమ|మం|బుధ|గురు|శుక్ర|శని)/i,
                wide: /^(ఆదివారం|సోమవారం|మంగళవారం|బుధవారం|గురువారం|శుక్రవారం|శనివారం)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^ఆ/i, /^సో/i, /^మ/i, /^బు/i, /^గు/i, /^శు/i, /^శ/i],
                any: [/^ఆది/i, /^సోమ/i, /^మం/i, /^బుధ/i, /^గురు/i, /^శుక్ర/i, /^శని/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow:
                  /^(పూర్వాహ్నం|అపరాహ్నం|అర్ధరాత్రి|మిట్టమధ్యాహ్నం|ఉదయం|మధ్యాహ్నం|సాయంత్రం|రాత్రి)/i,
                any: /^(పూర్వాహ్నం|అపరాహ్నం|అర్ధరాత్రి|మిట్టమధ్యాహ్నం|ఉదయం|మధ్యాహ్నం|సాయంత్రం|రాత్రి)/i,
              },
              defaultMatchWidth: 'any',
              parsePatterns: {
                any: {
                  am: /^పూర్వాహ్నం/i,
                  pm: /^అపరాహ్నం/i,
                  midnight: /^అర్ధ/i,
                  noon: /^మిట్ట/i,
                  morning: /ఉదయం/i,
                  afternoon: /మధ్యాహ్నం/i,
                  evening: /సాయంత్రం/i,
                  night: /రాత్రి/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/index.js': (
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
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'te',
          formatDistance: _index.default,
          formatLong: _index2.default,
          formatRelative: _index3.default,
          localize: _index4.default,
          match: _index5.default,
          options: { weekStartsOn: 0, firstWeekContainsDate: 1 },
        };
      (exports.default = _default), (module.exports = exports.default);
    },
  },
]);
