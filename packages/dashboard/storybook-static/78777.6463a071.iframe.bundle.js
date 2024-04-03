'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [78777],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ms/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'kurang dari 1 saat', other: 'kurang dari {{count}} saat' },
            xSeconds: { one: '1 saat', other: '{{count}} saat' },
            halfAMinute: 'setengah minit',
            lessThanXMinutes: { one: 'kurang dari 1 minit', other: 'kurang dari {{count}} minit' },
            xMinutes: { one: '1 minit', other: '{{count}} minit' },
            aboutXHours: { one: 'sekitar 1 jam', other: 'sekitar {{count}} jam' },
            xHours: { one: '1 jam', other: '{{count}} jam' },
            xDays: { one: '1 hari', other: '{{count}} hari' },
            aboutXWeeks: { one: 'sekitar 1 minggu', other: 'sekitar {{count}} minggu' },
            xWeeks: { one: '1 minggu', other: '{{count}} minggu' },
            aboutXMonths: { one: 'sekitar 1 bulan', other: 'sekitar {{count}} bulan' },
            xMonths: { one: '1 bulan', other: '{{count}} bulan' },
            aboutXYears: { one: 'sekitar 1 tahun', other: 'sekitar {{count}} tahun' },
            xYears: { one: '1 tahun', other: '{{count}} tahun' },
            overXYears: { one: 'lebih dari 1 tahun', other: 'lebih dari {{count}} tahun' },
            almostXYears: { one: 'hampir 1 tahun', other: 'hampir {{count}} tahun' },
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
                  ? 'dalam masa ' + result
                  : result + ' yang lalu'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
