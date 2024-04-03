'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [44726],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'ավելի քիչ քան 1 վայրկյան',
              other: 'ավելի քիչ քան {{count}} վայրկյան',
            },
            xSeconds: { one: '1 վայրկյան', other: '{{count}} վայրկյան' },
            halfAMinute: 'կես րոպե',
            lessThanXMinutes: {
              one: 'ավելի քիչ քան 1 րոպե',
              other: 'ավելի քիչ քան {{count}} րոպե',
            },
            xMinutes: { one: '1 րոպե', other: '{{count}} րոպե' },
            aboutXHours: { one: 'մոտ 1 ժամ', other: 'մոտ {{count}} ժամ' },
            xHours: { one: '1 ժամ', other: '{{count}} ժամ' },
            xDays: { one: '1 օր', other: '{{count}} օր' },
            aboutXWeeks: { one: 'մոտ 1 շաբաթ', other: 'մոտ {{count}} շաբաթ' },
            xWeeks: { one: '1 շաբաթ', other: '{{count}} շաբաթ' },
            aboutXMonths: { one: 'մոտ 1 ամիս', other: 'մոտ {{count}} ամիս' },
            xMonths: { one: '1 ամիս', other: '{{count}} ամիս' },
            aboutXYears: { one: 'մոտ 1 տարի', other: 'մոտ {{count}} տարի' },
            xYears: { one: '1 տարի', other: '{{count}} տարի' },
            overXYears: { one: 'ավելի քան 1 տարի', other: 'ավելի քան {{count}} տարի' },
            almostXYears: { one: 'համարյա 1 տարի', other: 'համարյա {{count}} տարի' },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? result + ' հետո'
                  : result + ' առաջ'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
