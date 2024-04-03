'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [69742, 388, 80951],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatDistance/index.js':
      (module, exports) => {
        function declension(scheme, count) {
          if (void 0 !== scheme.one && 1 === count) return scheme.one;
          var rem10 = count % 10,
            rem100 = count % 100;
          return 1 === rem10 && 11 !== rem100
            ? scheme.singularNominative.replace('{{count}}', String(count))
            : rem10 >= 2 && rem10 <= 4 && (rem100 < 10 || rem100 > 20)
              ? scheme.singularGenitive.replace('{{count}}', String(count))
              : scheme.pluralGenitive.replace('{{count}}', String(count));
        }
        function buildLocalizeTokenFn(scheme) {
          return function (count, options) {
            return options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? scheme.future
                  ? declension(scheme.future, count)
                  : 'праз ' + declension(scheme.regular, count)
                : scheme.past
                  ? declension(scheme.past, count)
                  : declension(scheme.regular, count) + ' таму'
              : declension(scheme.regular, count);
          };
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: buildLocalizeTokenFn({
              regular: {
                one: 'менш за секунду',
                singularNominative: 'менш за {{count}} секунду',
                singularGenitive: 'менш за {{count}} секунды',
                pluralGenitive: 'менш за {{count}} секунд',
              },
              future: {
                one: 'менш, чым праз секунду',
                singularNominative: 'менш, чым праз {{count}} секунду',
                singularGenitive: 'менш, чым праз {{count}} секунды',
                pluralGenitive: 'менш, чым праз {{count}} секунд',
              },
            }),
            xSeconds: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} секунда',
                singularGenitive: '{{count}} секунды',
                pluralGenitive: '{{count}} секунд',
              },
              past: {
                singularNominative: '{{count}} секунду таму',
                singularGenitive: '{{count}} секунды таму',
                pluralGenitive: '{{count}} секунд таму',
              },
              future: {
                singularNominative: 'праз {{count}} секунду',
                singularGenitive: 'праз {{count}} секунды',
                pluralGenitive: 'праз {{count}} секунд',
              },
            }),
            halfAMinute: function halfAMinute(_, options) {
              return options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'праз паўхвіліны'
                  : 'паўхвіліны таму'
                : 'паўхвіліны';
            },
            lessThanXMinutes: buildLocalizeTokenFn({
              regular: {
                one: 'менш за хвіліну',
                singularNominative: 'менш за {{count}} хвіліну',
                singularGenitive: 'менш за {{count}} хвіліны',
                pluralGenitive: 'менш за {{count}} хвілін',
              },
              future: {
                one: 'менш, чым праз хвіліну',
                singularNominative: 'менш, чым праз {{count}} хвіліну',
                singularGenitive: 'менш, чым праз {{count}} хвіліны',
                pluralGenitive: 'менш, чым праз {{count}} хвілін',
              },
            }),
            xMinutes: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} хвіліна',
                singularGenitive: '{{count}} хвіліны',
                pluralGenitive: '{{count}} хвілін',
              },
              past: {
                singularNominative: '{{count}} хвіліну таму',
                singularGenitive: '{{count}} хвіліны таму',
                pluralGenitive: '{{count}} хвілін таму',
              },
              future: {
                singularNominative: 'праз {{count}} хвіліну',
                singularGenitive: 'праз {{count}} хвіліны',
                pluralGenitive: 'праз {{count}} хвілін',
              },
            }),
            aboutXHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} гадзіны',
                singularGenitive: 'каля {{count}} гадзін',
                pluralGenitive: 'каля {{count}} гадзін',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} гадзіну',
                singularGenitive: 'прыблізна праз {{count}} гадзіны',
                pluralGenitive: 'прыблізна праз {{count}} гадзін',
              },
            }),
            xHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} гадзіна',
                singularGenitive: '{{count}} гадзіны',
                pluralGenitive: '{{count}} гадзін',
              },
              past: {
                singularNominative: '{{count}} гадзіну таму',
                singularGenitive: '{{count}} гадзіны таму',
                pluralGenitive: '{{count}} гадзін таму',
              },
              future: {
                singularNominative: 'праз {{count}} гадзіну',
                singularGenitive: 'праз {{count}} гадзіны',
                pluralGenitive: 'праз {{count}} гадзін',
              },
            }),
            xDays: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} дзень',
                singularGenitive: '{{count}} дні',
                pluralGenitive: '{{count}} дзён',
              },
            }),
            aboutXWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} месяца',
                singularGenitive: 'каля {{count}} месяцаў',
                pluralGenitive: 'каля {{count}} месяцаў',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} месяц',
                singularGenitive: 'прыблізна праз {{count}} месяцы',
                pluralGenitive: 'прыблізна праз {{count}} месяцаў',
              },
            }),
            xWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} месяц',
                singularGenitive: '{{count}} месяцы',
                pluralGenitive: '{{count}} месяцаў',
              },
            }),
            aboutXMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} месяца',
                singularGenitive: 'каля {{count}} месяцаў',
                pluralGenitive: 'каля {{count}} месяцаў',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} месяц',
                singularGenitive: 'прыблізна праз {{count}} месяцы',
                pluralGenitive: 'прыблізна праз {{count}} месяцаў',
              },
            }),
            xMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} месяц',
                singularGenitive: '{{count}} месяцы',
                pluralGenitive: '{{count}} месяцаў',
              },
            }),
            aboutXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} года',
                singularGenitive: 'каля {{count}} гадоў',
                pluralGenitive: 'каля {{count}} гадоў',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} год',
                singularGenitive: 'прыблізна праз {{count}} гады',
                pluralGenitive: 'прыблізна праз {{count}} гадоў',
              },
            }),
            xYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} год',
                singularGenitive: '{{count}} гады',
                pluralGenitive: '{{count}} гадоў',
              },
            }),
            overXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'больш за {{count}} год',
                singularGenitive: 'больш за {{count}} гады',
                pluralGenitive: 'больш за {{count}} гадоў',
              },
              future: {
                singularNominative: 'больш, чым праз {{count}} год',
                singularGenitive: 'больш, чым праз {{count}} гады',
                pluralGenitive: 'больш, чым праз {{count}} гадоў',
              },
            }),
            almostXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'амаль {{count}} год',
                singularGenitive: 'амаль {{count}} гады',
                pluralGenitive: 'амаль {{count}} гадоў',
              },
              future: {
                singularNominative: 'амаль праз {{count}} год',
                singularGenitive: 'амаль праз {{count}} гады',
                pluralGenitive: 'амаль праз {{count}} гадоў',
              },
            }),
          },
          _default = function formatDistance(token, count, options) {
            return (options = options || {}), formatDistanceLocale[token](count, options);
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatLong/index.js':
      (module, exports, __webpack_require__) => {
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
                full: "EEEE, d MMMM y 'г.'",
                long: "d MMMM y 'г.'",
                medium: "d MMM y 'г.'",
                short: 'dd.MM.y',
              },
              defaultWidth: 'full',
            }),
            time: (0, _index.default)({
              formats: {
                full: 'H:mm:ss zzzz',
                long: 'H:mm:ss z',
                medium: 'H:mm:ss',
                short: 'H:mm',
              },
              defaultWidth: 'full',
            }),
            dateTime: (0, _index.default)({
              formats: { any: '{{date}}, {{time}}' },
              defaultWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatRelative/index.js':
      (module, exports, __webpack_require__) => {
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/index.js',
          ),
          _index2 = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/isSameUTCWeek/index.js',
            ),
          ),
          accusativeWeekdays = [
            'нядзелю',
            'панядзелак',
            'аўторак',
            'сераду',
            'чацвер',
            'пятніцу',
            'суботу',
          ];
        function thisWeek(day) {
          return "'у " + accusativeWeekdays[day] + " а' p";
        }
        var formatRelativeLocale = {
            lastWeek: function lastWeekFormat(dirtyDate, baseDate, options) {
              var date = (0, _index.toDate)(dirtyDate),
                day = date.getUTCDay();
              return (0, _index2.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function lastWeek(day) {
                    var weekday = accusativeWeekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 5:
                      case 6:
                        return "'у мінулую " + weekday + " а' p";
                      case 1:
                      case 2:
                      case 4:
                        return "'у мінулы " + weekday + " а' p";
                    }
                  })(day);
            },
            yesterday: "'учора а' p",
            today: "'сёння а' p",
            tomorrow: "'заўтра а' p",
            nextWeek: function nextWeekFormat(dirtyDate, baseDate, options) {
              var date = (0, _index.toDate)(dirtyDate),
                day = date.getUTCDay();
              return (0, _index2.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function nextWeek(day) {
                    var weekday = accusativeWeekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 5:
                      case 6:
                        return "'у наступную " + weekday + " а' p";
                      case 1:
                      case 2:
                      case 4:
                        return "'у наступны " + weekday + " а' p";
                    }
                  })(day);
            },
            other: 'P',
          },
          _default = function formatRelative(token, date, baseDate, options) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date, baseDate, options) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/localize/index.js':
      (module, exports, __webpack_require__) => {
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
            ordinalNumber: function ordinalNumber(dirtyNumber, options) {
              var unit = String(null == options ? void 0 : options.unit),
                number = Number(dirtyNumber);
              return (
                number +
                ('date' === unit
                  ? '-га'
                  : 'hour' === unit || 'minute' === unit || 'second' === unit
                    ? '-я'
                    : (number % 10 != 2 && number % 10 != 3) ||
                        number % 100 == 12 ||
                        number % 100 == 13
                      ? '-ы'
                      : '-і')
              );
            },
            era: (0, _index.default)({
              values: {
                narrow: ['да н.э.', 'н.э.'],
                abbreviated: ['да н. э.', 'н. э.'],
                wide: ['да нашай эры', 'нашай эры'],
              },
              defaultWidth: 'wide',
            }),
            quarter: (0, _index.default)({
              values: {
                narrow: ['1', '2', '3', '4'],
                abbreviated: ['1-ы кв.', '2-і кв.', '3-і кв.', '4-ы кв.'],
                wide: ['1-ы квартал', '2-і квартал', '3-і квартал', '4-ы квартал'],
              },
              defaultWidth: 'wide',
              argumentCallback: function argumentCallback(quarter) {
                return quarter - 1;
              },
            }),
            month: (0, _index.default)({
              values: {
                narrow: ['С', 'Л', 'С', 'К', 'М', 'Ч', 'Л', 'Ж', 'В', 'К', 'Л', 'С'],
                abbreviated: [
                  'студз.',
                  'лют.',
                  'сак.',
                  'крас.',
                  'май',
                  'чэрв.',
                  'ліп.',
                  'жн.',
                  'вер.',
                  'кастр.',
                  'ліст.',
                  'снеж.',
                ],
                wide: [
                  'студзень',
                  'люты',
                  'сакавік',
                  'красавік',
                  'май',
                  'чэрвень',
                  'ліпень',
                  'жнівень',
                  'верасень',
                  'кастрычнік',
                  'лістапад',
                  'снежань',
                ],
              },
              defaultWidth: 'wide',
              formattingValues: {
                narrow: ['С', 'Л', 'С', 'К', 'М', 'Ч', 'Л', 'Ж', 'В', 'К', 'Л', 'С'],
                abbreviated: [
                  'студз.',
                  'лют.',
                  'сак.',
                  'крас.',
                  'мая',
                  'чэрв.',
                  'ліп.',
                  'жн.',
                  'вер.',
                  'кастр.',
                  'ліст.',
                  'снеж.',
                ],
                wide: [
                  'студзеня',
                  'лютага',
                  'сакавіка',
                  'красавіка',
                  'мая',
                  'чэрвеня',
                  'ліпеня',
                  'жніўня',
                  'верасня',
                  'кастрычніка',
                  'лістапада',
                  'снежня',
                ],
              },
              defaultFormattingWidth: 'wide',
            }),
            day: (0, _index.default)({
              values: {
                narrow: ['Н', 'П', 'А', 'С', 'Ч', 'П', 'С'],
                short: ['нд', 'пн', 'аў', 'ср', 'чц', 'пт', 'сб'],
                abbreviated: ['нядз', 'пан', 'аўт', 'сер', 'чац', 'пят', 'суб'],
                wide: ['нядзеля', 'панядзелак', 'аўторак', 'серада', 'чацвер', 'пятніца', 'субота'],
              },
              defaultWidth: 'wide',
            }),
            dayPeriod: (0, _index.default)({
              values: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дзень',
                  evening: 'веч.',
                  night: 'ноч',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дзень',
                  evening: 'веч.',
                  night: 'ноч',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўнач',
                  noon: 'поўдзень',
                  morning: 'раніца',
                  afternoon: 'дзень',
                  evening: 'вечар',
                  night: 'ноч',
                },
              },
              defaultWidth: 'any',
              formattingValues: {
                narrow: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночы',
                },
                abbreviated: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўн.',
                  noon: 'поўд.',
                  morning: 'ран.',
                  afternoon: 'дня',
                  evening: 'веч.',
                  night: 'ночы',
                },
                wide: {
                  am: 'ДП',
                  pm: 'ПП',
                  midnight: 'поўнач',
                  noon: 'поўдзень',
                  morning: 'раніцы',
                  afternoon: 'дня',
                  evening: 'вечара',
                  night: 'ночы',
                },
              },
              defaultFormattingWidth: 'wide',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/match/index.js':
      (module, exports, __webpack_require__) => {
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
              matchPattern: /^(\d+)(-?(е|я|га|і|ы|ае|ая|яя|шы|гі|ці|ты|мы))?/i,
              parsePattern: /\d+/i,
              valueCallback: function valueCallback(value) {
                return parseInt(value, 10);
              },
            }),
            era: (0, _index.default)({
              matchPatterns: {
                narrow: /^((да )?н\.?\s?э\.?)/i,
                abbreviated: /^((да )?н\.?\s?э\.?)/i,
                wide: /^(да нашай эры|нашай эры|наша эра)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: { any: [/^д/i, /^н/i] },
              defaultParseWidth: 'any',
            }),
            quarter: (0, _index.default)({
              matchPatterns: {
                narrow: /^[1234]/i,
                abbreviated: /^[1234](-?[ыі]?)? кв.?/i,
                wide: /^[1234](-?[ыі]?)? квартал/i,
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
                narrow: /^[слкмчжв]/i,
                abbreviated: /^(студз|лют|сак|крас|ма[йя]|чэрв|ліп|жн|вер|кастр|ліст|снеж)\.?/i,
                wide: /^(студзен[ья]|лют(ы|ага)|сакавіка?|красавіка?|ма[йя]|чэрвен[ья]|ліпен[ья]|жні(вень|ўня)|верас(ень|ня)|кастрычніка?|лістапада?|снеж(ань|ня))/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [
                  /^с/i,
                  /^л/i,
                  /^с/i,
                  /^к/i,
                  /^м/i,
                  /^ч/i,
                  /^л/i,
                  /^ж/i,
                  /^в/i,
                  /^к/i,
                  /^л/i,
                  /^с/i,
                ],
                any: [
                  /^ст/i,
                  /^лю/i,
                  /^са/i,
                  /^кр/i,
                  /^ма/i,
                  /^ч/i,
                  /^ліп/i,
                  /^ж/i,
                  /^в/i,
                  /^ка/i,
                  /^ліс/i,
                  /^сн/i,
                ],
              },
              defaultParseWidth: 'any',
            }),
            day: (0, _index.default)({
              matchPatterns: {
                narrow: /^[нпасч]/i,
                short: /^(нд|ня|пн|па|аў|ат|ср|се|чц|ча|пт|пя|сб|су)\.?/i,
                abbreviated: /^(нядз?|ндз|пнд|пан|аўт|срд|сер|чцв|чац|птн|пят|суб).?/i,
                wide: /^(нядзел[яі]|панядзел(ак|ка)|аўтор(ак|ка)|серад[аы]|чацв(ер|ярга)|пятніц[аы]|субот[аы])/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                narrow: [/^н/i, /^п/i, /^а/i, /^с/i, /^ч/i, /^п/i, /^с/i],
                any: [/^н/i, /^п[ан]/i, /^а/i, /^с[ер]/i, /^ч/i, /^п[ят]/i, /^с[уб]/i],
              },
              defaultParseWidth: 'any',
            }),
            dayPeriod: (0, _index.default)({
              matchPatterns: {
                narrow: /^([дп]п|поўн\.?|поўд\.?|ран\.?|дзень|дня|веч\.?|ночы?)/i,
                abbreviated: /^([дп]п|поўн\.?|поўд\.?|ран\.?|дзень|дня|веч\.?|ночы?)/i,
                wide: /^([дп]п|поўнач|поўдзень|раніц[аы]|дзень|дня|вечара?|ночы?)/i,
              },
              defaultMatchWidth: 'wide',
              parsePatterns: {
                any: {
                  am: /^дп/i,
                  pm: /^пп/i,
                  midnight: /^поўн/i,
                  noon: /^поўд/i,
                  morning: /^р/i,
                  afternoon: /^д[зн]/i,
                  evening: /^в/i,
                  night: /^н/i,
                },
              },
              defaultParseWidth: 'any',
            }),
          };
        (exports.default = _default), (module.exports = exports.default);
      },
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/index.js': (
      module,
      exports,
      __webpack_require__,
    ) => {
      var _interopRequireDefault = __webpack_require__(
        '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
      ).default;
      Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
      var _index = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatDistance/index.js',
          ),
        ),
        _index2 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatLong/index.js',
          ),
        ),
        _index3 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/formatRelative/index.js',
          ),
        ),
        _index4 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/localize/index.js',
          ),
        ),
        _index5 = _interopRequireDefault(
          __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be/_lib/match/index.js',
          ),
        ),
        _default = {
          code: 'be',
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
