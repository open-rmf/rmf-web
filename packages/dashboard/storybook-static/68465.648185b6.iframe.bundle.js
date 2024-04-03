'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [68465],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'sekunddan kam', other: '{{count}} sekunddan kam' },
            xSeconds: { one: '1 sekund', other: '{{count}} sekund' },
            halfAMinute: 'yarim minut',
            lessThanXMinutes: { one: 'bir minutdan kam', other: '{{count}} minutdan kam' },
            xMinutes: { one: '1 minut', other: '{{count}} minut' },
            aboutXHours: { one: 'tahminan 1 soat', other: 'tahminan {{count}} soat' },
            xHours: { one: '1 soat', other: '{{count}} soat' },
            xDays: { one: '1 kun', other: '{{count}} kun' },
            aboutXWeeks: { one: 'tahminan 1 hafta', other: 'tahminan {{count}} hafta' },
            xWeeks: { one: '1 hafta', other: '{{count}} hafta' },
            aboutXMonths: { one: 'tahminan 1 oy', other: 'tahminan {{count}} oy' },
            xMonths: { one: '1 oy', other: '{{count}} oy' },
            aboutXYears: { one: 'tahminan 1 yil', other: 'tahminan {{count}} yil' },
            xYears: { one: '1 yil', other: '{{count}} yil' },
            overXYears: { one: "1 yildan ko'p", other: "{{count}} yildan ko'p" },
            almostXYears: { one: 'deyarli 1 yil', other: 'deyarli {{count}} yil' },
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
                  ? result + ' dan keyin'
                  : result + ' oldin'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
