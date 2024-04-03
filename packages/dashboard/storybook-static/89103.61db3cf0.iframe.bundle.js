'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [89103],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'bir saniyeden az', other: '{{count}} saniyeden az' },
            xSeconds: { one: '1 saniye', other: '{{count}} saniye' },
            halfAMinute: 'yarım dakika',
            lessThanXMinutes: { one: 'bir dakikadan az', other: '{{count}} dakikadan az' },
            xMinutes: { one: '1 dakika', other: '{{count}} dakika' },
            aboutXHours: { one: 'yaklaşık 1 saat', other: 'yaklaşık {{count}} saat' },
            xHours: { one: '1 saat', other: '{{count}} saat' },
            xDays: { one: '1 gün', other: '{{count}} gün' },
            aboutXWeeks: { one: 'yaklaşık 1 hafta', other: 'yaklaşık {{count}} hafta' },
            xWeeks: { one: '1 hafta', other: '{{count}} hafta' },
            aboutXMonths: { one: 'yaklaşık 1 ay', other: 'yaklaşık {{count}} ay' },
            xMonths: { one: '1 ay', other: '{{count}} ay' },
            aboutXYears: { one: 'yaklaşık 1 yıl', other: 'yaklaşık {{count}} yıl' },
            xYears: { one: '1 yıl', other: '{{count}} yıl' },
            overXYears: { one: '1 yıldan fazla', other: '{{count}} yıldan fazla' },
            almostXYears: { one: 'neredeyse 1 yıl', other: 'neredeyse {{count}} yıl' },
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
                    : tokenValue.other.replace('{{count}}', count.toString())),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? result + ' sonra'
                  : result + ' önce'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
