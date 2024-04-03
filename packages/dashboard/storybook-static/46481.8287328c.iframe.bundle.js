'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [46481],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lt/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var translations = {
            xseconds_other: 'sekundė_sekundžių_sekundes',
            xminutes_one: 'minutė_minutės_minutę',
            xminutes_other: 'minutės_minučių_minutes',
            xhours_one: 'valanda_valandos_valandą',
            xhours_other: 'valandos_valandų_valandas',
            xdays_one: 'diena_dienos_dieną',
            xdays_other: 'dienos_dienų_dienas',
            xweeks_one: 'savaitė_savaitės_savaitę',
            xweeks_other: 'savaitės_savaičių_savaites',
            xmonths_one: 'mėnuo_mėnesio_mėnesį',
            xmonths_other: 'mėnesiai_mėnesių_mėnesius',
            xyears_one: 'metai_metų_metus',
            xyears_other: 'metai_metų_metus',
            about: 'apie',
            over: 'daugiau nei',
            almost: 'beveik',
            lessthan: 'mažiau nei',
          },
          translateSeconds = function translateSeconds(_number, addSuffix, _key, isFuture) {
            return addSuffix
              ? isFuture
                ? 'kelių sekundžių'
                : 'kelias sekundes'
              : 'kelios sekundės';
          },
          translateSingular = function translateSingular(_number, addSuffix, key, isFuture) {
            return addSuffix ? (isFuture ? forms(key)[1] : forms(key)[2]) : forms(key)[0];
          },
          translate = function translate(number, addSuffix, key, isFuture) {
            var result = number + ' ';
            return 1 === number
              ? result + translateSingular(0, addSuffix, key, isFuture)
              : addSuffix
                ? isFuture
                  ? result + forms(key)[1]
                  : result + (special(number) ? forms(key)[1] : forms(key)[2])
                : result + (special(number) ? forms(key)[1] : forms(key)[0]);
          };
        function special(number) {
          return number % 10 == 0 || (number > 10 && number < 20);
        }
        function forms(key) {
          return translations[key].split('_');
        }
        var formatDistanceLocale = {
            lessThanXSeconds: { one: translateSeconds, other: translate },
            xSeconds: { one: translateSeconds, other: translate },
            halfAMinute: 'pusė minutės',
            lessThanXMinutes: { one: translateSingular, other: translate },
            xMinutes: { one: translateSingular, other: translate },
            aboutXHours: { one: translateSingular, other: translate },
            xHours: { one: translateSingular, other: translate },
            xDays: { one: translateSingular, other: translate },
            aboutXWeeks: { one: translateSingular, other: translate },
            xWeeks: { one: translateSingular, other: translate },
            aboutXMonths: { one: translateSingular, other: translate },
            xMonths: { one: translateSingular, other: translate },
            aboutXYears: { one: translateSingular, other: translate },
            xYears: { one: translateSingular, other: translate },
            overXYears: { one: translateSingular, other: translate },
            almostXYears: { one: translateSingular, other: translate },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              adverb = token.match(/about|over|almost|lessthan/i),
              unit = adverb ? token.replace(adverb[0], '') : token,
              isFuture =
                void 0 !== (null == options ? void 0 : options.comparison) &&
                options.comparison > 0,
              tokenValue = formatDistanceLocale[token];
            if (
              ((result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one(
                        count,
                        !0 === (null == options ? void 0 : options.addSuffix),
                        unit.toLowerCase() + '_one',
                        isFuture,
                      )
                    : tokenValue.other(
                        count,
                        !0 === (null == options ? void 0 : options.addSuffix),
                        unit.toLowerCase() + '_other',
                        isFuture,
                      )),
              adverb)
            ) {
              var _key2 = adverb[0].toLowerCase();
              result = translations[_key2] + ' ' + result;
            }
            return null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? 'po ' + result
                : 'prieš ' + result
              : result;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
