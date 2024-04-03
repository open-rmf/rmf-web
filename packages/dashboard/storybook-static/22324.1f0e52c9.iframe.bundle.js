'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [22324],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'inqas minn sekonda', other: 'inqas minn {{count}} sekondi' },
            xSeconds: { one: 'sekonda', other: '{{count}} sekondi' },
            halfAMinute: 'nofs minuta',
            lessThanXMinutes: { one: 'inqas minn minuta', other: 'inqas minn {{count}} minuti' },
            xMinutes: { one: 'minuta', other: '{{count}} minuti' },
            aboutXHours: { one: 'madwar siegħa', other: 'madwar {{count}} siegħat' },
            xHours: { one: 'siegħa', other: '{{count}} siegħat' },
            xDays: { one: 'ġurnata', other: '{{count}} ġranet' },
            aboutXWeeks: { one: 'madwar ġimgħa', other: 'madwar {{count}} ġimgħat' },
            xWeeks: { one: 'ġimgħa', other: '{{count}} ġimgħat' },
            aboutXMonths: { one: 'madwar xahar', other: 'madwar {{count}} xhur' },
            xMonths: { one: 'xahar', other: '{{count}} xhur' },
            aboutXYears: {
              one: 'madwar sena',
              two: 'madwar sentejn',
              other: 'madwar {{count}} snin',
            },
            xYears: { one: 'sena', two: 'sentejn', other: '{{count}} snin' },
            overXYears: {
              one: 'aktar minn sena',
              two: 'aktar minn sentejn',
              other: 'aktar minn {{count}} snin',
            },
            almostXYears: {
              one: 'kważi sena',
              two: 'kważi sentejn',
              other: 'kważi {{count}} snin',
            },
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
                    : 2 === count && tokenValue.two
                      ? tokenValue.two
                      : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? "f'" + result
                  : result + ' ilu'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
