'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [61250],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ro/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'mai puțin de o secundă',
              other: 'mai puțin de {{count}} secunde',
            },
            xSeconds: { one: '1 secundă', other: '{{count}} secunde' },
            halfAMinute: 'jumătate de minut',
            lessThanXMinutes: {
              one: 'mai puțin de un minut',
              other: 'mai puțin de {{count}} minute',
            },
            xMinutes: { one: '1 minut', other: '{{count}} minute' },
            aboutXHours: { one: 'circa 1 oră', other: 'circa {{count}} ore' },
            xHours: { one: '1 oră', other: '{{count}} ore' },
            xDays: { one: '1 zi', other: '{{count}} zile' },
            aboutXWeeks: { one: 'circa o săptămână', other: 'circa {{count}} săptămâni' },
            xWeeks: { one: '1 săptămână', other: '{{count}} săptămâni' },
            aboutXMonths: { one: 'circa 1 lună', other: 'circa {{count}} luni' },
            xMonths: { one: '1 lună', other: '{{count}} luni' },
            aboutXYears: { one: 'circa 1 an', other: 'circa {{count}} ani' },
            xYears: { one: '1 an', other: '{{count}} ani' },
            overXYears: { one: 'peste 1 an', other: 'peste {{count}} ani' },
            almostXYears: { one: 'aproape 1 an', other: 'aproape {{count}} ani' },
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
                  ? 'în ' + result
                  : result + ' în urmă'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
