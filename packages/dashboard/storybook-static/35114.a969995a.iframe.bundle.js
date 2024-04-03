'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [35114],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'bir saniyədən az', other: '{{count}} bir saniyədən az' },
            xSeconds: { one: '1 saniyə', other: '{{count}} saniyə' },
            halfAMinute: 'yarım dəqiqə',
            lessThanXMinutes: { one: 'bir dəqiqədən az', other: '{{count}} bir dəqiqədən az' },
            xMinutes: { one: 'bir dəqiqə', other: '{{count}} dəqiqə' },
            aboutXHours: { one: 'təxminən 1 saat', other: 'təxminən {{count}} saat' },
            xHours: { one: '1 saat', other: '{{count}} saat' },
            xDays: { one: '1 gün', other: '{{count}} gün' },
            aboutXWeeks: { one: 'təxminən 1 həftə', other: 'təxminən {{count}} həftə' },
            xWeeks: { one: '1 həftə', other: '{{count}} həftə' },
            aboutXMonths: { one: 'təxminən 1 ay', other: 'təxminən {{count}} ay' },
            xMonths: { one: '1 ay', other: '{{count}} ay' },
            aboutXYears: { one: 'təxminən 1 il', other: 'təxminən {{count}} il' },
            xYears: { one: '1 il', other: '{{count}} il' },
            overXYears: { one: '1 ildən çox', other: '{{count}} ildən çox' },
            almostXYears: { one: 'demək olar ki 1 il', other: 'demək olar ki {{count}} il' },
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
                  ? result + ' sonra'
                  : result + ' əvvəl'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
