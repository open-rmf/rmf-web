'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [9381],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nb/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'mindre enn ett sekund',
              other: 'mindre enn {{count}} sekunder',
            },
            xSeconds: { one: 'ett sekund', other: '{{count}} sekunder' },
            halfAMinute: 'et halvt minutt',
            lessThanXMinutes: {
              one: 'mindre enn ett minutt',
              other: 'mindre enn {{count}} minutter',
            },
            xMinutes: { one: 'ett minutt', other: '{{count}} minutter' },
            aboutXHours: { one: 'omtrent en time', other: 'omtrent {{count}} timer' },
            xHours: { one: 'en time', other: '{{count}} timer' },
            xDays: { one: 'en dag', other: '{{count}} dager' },
            aboutXWeeks: { one: 'omtrent en uke', other: 'omtrent {{count}} uker' },
            xWeeks: { one: 'en uke', other: '{{count}} uker' },
            aboutXMonths: { one: 'omtrent en måned', other: 'omtrent {{count}} måneder' },
            xMonths: { one: 'en måned', other: '{{count}} måneder' },
            aboutXYears: { one: 'omtrent ett år', other: 'omtrent {{count}} år' },
            xYears: { one: 'ett år', other: '{{count}} år' },
            overXYears: { one: 'over ett år', other: 'over {{count}} år' },
            almostXYears: { one: 'nesten ett år', other: 'nesten {{count}} år' },
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
