'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [60519],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/lb/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              standalone: { one: 'manner wéi eng Sekonn', other: 'manner wéi {{count}} Sekonnen' },
              withPreposition: {
                one: 'manner wéi enger Sekonn',
                other: 'manner wéi {{count}} Sekonnen',
              },
            },
            xSeconds: {
              standalone: { one: 'eng Sekonn', other: '{{count}} Sekonnen' },
              withPreposition: { one: 'enger Sekonn', other: '{{count}} Sekonnen' },
            },
            halfAMinute: {
              standalone: 'eng hallef Minutt',
              withPreposition: 'enger hallwer Minutt',
            },
            lessThanXMinutes: {
              standalone: { one: 'manner wéi eng Minutt', other: 'manner wéi {{count}} Minutten' },
              withPreposition: {
                one: 'manner wéi enger Minutt',
                other: 'manner wéi {{count}} Minutten',
              },
            },
            xMinutes: {
              standalone: { one: 'eng Minutt', other: '{{count}} Minutten' },
              withPreposition: { one: 'enger Minutt', other: '{{count}} Minutten' },
            },
            aboutXHours: {
              standalone: { one: 'ongeféier eng Stonn', other: 'ongeféier {{count}} Stonnen' },
              withPreposition: {
                one: 'ongeféier enger Stonn',
                other: 'ongeféier {{count}} Stonnen',
              },
            },
            xHours: {
              standalone: { one: 'eng Stonn', other: '{{count}} Stonnen' },
              withPreposition: { one: 'enger Stonn', other: '{{count}} Stonnen' },
            },
            xDays: {
              standalone: { one: 'een Dag', other: '{{count}} Deeg' },
              withPreposition: { one: 'engem Dag', other: '{{count}} Deeg' },
            },
            aboutXWeeks: {
              standalone: { one: 'ongeféier eng Woch', other: 'ongeféier {{count}} Wochen' },
              withPreposition: {
                one: 'ongeféier enger Woche',
                other: 'ongeféier {{count}} Wochen',
              },
            },
            xWeeks: {
              standalone: { one: 'eng Woch', other: '{{count}} Wochen' },
              withPreposition: { one: 'enger Woch', other: '{{count}} Wochen' },
            },
            aboutXMonths: {
              standalone: { one: 'ongeféier ee Mount', other: 'ongeféier {{count}} Méint' },
              withPreposition: { one: 'ongeféier engem Mount', other: 'ongeféier {{count}} Méint' },
            },
            xMonths: {
              standalone: { one: 'ee Mount', other: '{{count}} Méint' },
              withPreposition: { one: 'engem Mount', other: '{{count}} Méint' },
            },
            aboutXYears: {
              standalone: { one: 'ongeféier ee Joer', other: 'ongeféier {{count}} Joer' },
              withPreposition: { one: 'ongeféier engem Joer', other: 'ongeféier {{count}} Joer' },
            },
            xYears: {
              standalone: { one: 'ee Joer', other: '{{count}} Joer' },
              withPreposition: { one: 'engem Joer', other: '{{count}} Joer' },
            },
            overXYears: {
              standalone: { one: 'méi wéi ee Joer', other: 'méi wéi {{count}} Joer' },
              withPreposition: { one: 'méi wéi engem Joer', other: 'méi wéi {{count}} Joer' },
            },
            almostXYears: {
              standalone: { one: 'bal ee Joer', other: 'bal {{count}} Joer' },
              withPreposition: { one: 'bal engem Joer', other: 'bal {{count}} Joer' },
            },
          },
          EXCEPTION_CONSONANTS = ['d', 'h', 'n', 't', 'z'],
          VOWELS = ['a,', 'e', 'i', 'o', 'u'],
          DIGITS_SPOKEN_N_NEEDED = [0, 1, 2, 3, 8, 9],
          FIRST_TWO_DIGITS_SPOKEN_NO_N_NEEDED = [40, 50, 60, 70];
        function isFinalNNeeded(nextWords) {
          var firstLetter = nextWords.charAt(0).toLowerCase();
          if (-1 != VOWELS.indexOf(firstLetter) || -1 != EXCEPTION_CONSONANTS.indexOf(firstLetter))
            return !0;
          var firstWord = nextWords.split(' ')[0],
            number = parseInt(firstWord);
          return (
            !isNaN(number) &&
            -1 != DIGITS_SPOKEN_N_NEEDED.indexOf(number % 10) &&
            -1 == FIRST_TWO_DIGITS_SPOKEN_NO_N_NEEDED.indexOf(parseInt(firstWord.substring(0, 2)))
          );
        }
        var _default = function formatDistance(token, count, options) {
          var result,
            tokenValue = formatDistanceLocale[token],
            usageGroup =
              null != options && options.addSuffix
                ? tokenValue.withPreposition
                : tokenValue.standalone;
          return (
            (result =
              'string' == typeof usageGroup
                ? usageGroup
                : 1 === count
                  ? usageGroup.one
                  : usageGroup.other.replace('{{count}}', String(count))),
            null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? 'a' + (isFinalNNeeded(result) ? 'n' : '') + ' ' + result
                : 'viru' + (isFinalNNeeded(result) ? 'n' : '') + ' ' + result
              : result
          );
        };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
