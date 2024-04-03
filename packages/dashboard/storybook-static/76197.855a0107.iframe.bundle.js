'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [76197],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ht/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'mwens pase yon segond',
              other: 'mwens pase {{count}} segond',
            },
            xSeconds: { one: '1 segond', other: '{{count}} segond' },
            halfAMinute: '30 segond',
            lessThanXMinutes: { one: 'mwens pase yon minit', other: 'mwens pase {{count}} minit' },
            xMinutes: { one: '1 minit', other: '{{count}} minit' },
            aboutXHours: { one: 'anviwon inè', other: 'anviwon {{count}} è' },
            xHours: { one: '1 lè', other: '{{count}} lè' },
            xDays: { one: '1 jou', other: '{{count}} jou' },
            aboutXWeeks: { one: 'anviwon 1 semèn', other: 'anviwon {{count}} semèn' },
            xWeeks: { one: '1 semèn', other: '{{count}} semèn' },
            aboutXMonths: { one: 'anviwon 1 mwa', other: 'anviwon {{count}} mwa' },
            xMonths: { one: '1 mwa', other: '{{count}} mwa' },
            aboutXYears: { one: 'anviwon 1 an', other: 'anviwon {{count}} an' },
            xYears: { one: '1 an', other: '{{count}} an' },
            overXYears: { one: 'plis pase 1 an', other: 'plis pase {{count}} an' },
            almostXYears: { one: 'prèske 1 an', other: 'prèske {{count}} an' },
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
                  ? 'nan ' + result
                  : 'sa fè ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
