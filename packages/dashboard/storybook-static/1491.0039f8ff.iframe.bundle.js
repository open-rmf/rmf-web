'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [1491],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz-Cyrl/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: '1 сониядан кам', other: '{{count}} сониядан кам' },
            xSeconds: { one: '1 сония', other: '{{count}} сония' },
            halfAMinute: 'ярим дақиқа',
            lessThanXMinutes: { one: '1 дақиқадан кам', other: '{{count}} дақиқадан кам' },
            xMinutes: { one: '1 дақиқа', other: '{{count}} дақиқа' },
            aboutXHours: { one: 'тахминан 1 соат', other: 'тахминан {{count}} соат' },
            xHours: { one: '1 соат', other: '{{count}} соат' },
            xDays: { one: '1 кун', other: '{{count}} кун' },
            aboutXWeeks: { one: 'тахминан 1 хафта', other: 'тахминан {{count}} хафта' },
            xWeeks: { one: '1 хафта', other: '{{count}} хафта' },
            aboutXMonths: { one: 'тахминан 1 ой', other: 'тахминан {{count}} ой' },
            xMonths: { one: '1 ой', other: '{{count}} ой' },
            aboutXYears: { one: 'тахминан 1 йил', other: 'тахминан {{count}} йил' },
            xYears: { one: '1 йил', other: '{{count}} йил' },
            overXYears: { one: '1 йилдан кўп', other: '{{count}} йилдан кўп' },
            almostXYears: { one: 'деярли 1 йил', other: 'деярли {{count}} йил' },
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
                  ? result + 'дан кейин'
                  : result + ' олдин'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
