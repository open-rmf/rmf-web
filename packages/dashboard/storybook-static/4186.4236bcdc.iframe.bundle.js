'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [4186],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'mindre end ét sekund',
              other: 'mindre end {{count}} sekunder',
            },
            xSeconds: { one: '1 sekund', other: '{{count}} sekunder' },
            halfAMinute: 'ét halvt minut',
            lessThanXMinutes: {
              one: 'mindre end ét minut',
              other: 'mindre end {{count}} minutter',
            },
            xMinutes: { one: '1 minut', other: '{{count}} minutter' },
            aboutXHours: { one: 'cirka 1 time', other: 'cirka {{count}} timer' },
            xHours: { one: '1 time', other: '{{count}} timer' },
            xDays: { one: '1 dag', other: '{{count}} dage' },
            aboutXWeeks: { one: 'cirka 1 uge', other: 'cirka {{count}} uger' },
            xWeeks: { one: '1 uge', other: '{{count}} uger' },
            aboutXMonths: { one: 'cirka 1 måned', other: 'cirka {{count}} måneder' },
            xMonths: { one: '1 måned', other: '{{count}} måneder' },
            aboutXYears: { one: 'cirka 1 år', other: 'cirka {{count}} år' },
            xYears: { one: '1 år', other: '{{count}} år' },
            overXYears: { one: 'over 1 år', other: 'over {{count}} år' },
            almostXYears: { one: 'næsten 1 år', other: 'næsten {{count}} år' },
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
                  ? 'om ' + result
                  : result + ' siden'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
