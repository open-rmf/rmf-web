'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [44799],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nl-BE/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'minder dan een seconde',
              other: 'minder dan {{count}} seconden',
            },
            xSeconds: { one: '1 seconde', other: '{{count}} seconden' },
            halfAMinute: 'een halve minuut',
            lessThanXMinutes: {
              one: 'minder dan een minuut',
              other: 'minder dan {{count}} minuten',
            },
            xMinutes: { one: 'een minuut', other: '{{count}} minuten' },
            aboutXHours: { one: 'ongeveer 1 uur', other: 'ongeveer {{count}} uur' },
            xHours: { one: '1 uur', other: '{{count}} uur' },
            xDays: { one: '1 dag', other: '{{count}} dagen' },
            aboutXWeeks: { one: 'ongeveer 1 week', other: 'ongeveer {{count}} weken' },
            xWeeks: { one: '1 week', other: '{{count}} weken' },
            aboutXMonths: { one: 'ongeveer 1 maand', other: 'ongeveer {{count}} maanden' },
            xMonths: { one: '1 maand', other: '{{count}} maanden' },
            aboutXYears: { one: 'ongeveer 1 jaar', other: 'ongeveer {{count}} jaar' },
            xYears: { one: '1 jaar', other: '{{count}} jaar' },
            overXYears: { one: 'meer dan 1 jaar', other: 'meer dan {{count}} jaar' },
            almostXYears: { one: 'bijna 1 jaar', other: 'bijna {{count}} jaar' },
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
                  ? 'over ' + result
                  : result + ' geleden'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
