'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [59847],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'segundo bat baino gutxiago',
              other: '{{count}} segundo baino gutxiago',
            },
            xSeconds: { one: '1 segundo', other: '{{count}} segundo' },
            halfAMinute: 'minutu erdi',
            lessThanXMinutes: {
              one: 'minutu bat baino gutxiago',
              other: '{{count}} minutu baino gutxiago',
            },
            xMinutes: { one: '1 minutu', other: '{{count}} minutu' },
            aboutXHours: {
              one: '1 ordu gutxi gorabehera',
              other: '{{count}} ordu gutxi gorabehera',
            },
            xHours: { one: '1 ordu', other: '{{count}} ordu' },
            xDays: { one: '1 egun', other: '{{count}} egun' },
            aboutXWeeks: { one: 'aste 1 inguru', other: '{{count}} aste inguru' },
            xWeeks: { one: '1 aste', other: '{{count}} astean' },
            aboutXMonths: {
              one: '1 hilabete gutxi gorabehera',
              other: '{{count}} hilabete gutxi gorabehera',
            },
            xMonths: { one: '1 hilabete', other: '{{count}} hilabete' },
            aboutXYears: {
              one: '1 urte gutxi gorabehera',
              other: '{{count}} urte gutxi gorabehera',
            },
            xYears: { one: '1 urte', other: '{{count}} urte' },
            overXYears: { one: '1 urte baino gehiago', other: '{{count}} urte baino gehiago' },
            almostXYears: { one: 'ia 1 urte', other: 'ia {{count}} urte' },
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
                  ? 'en ' + result
                  : 'duela ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
