'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [3546],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'פחות משנייה',
              two: 'פחות משתי שניות',
              other: 'פחות מ־{{count}} שניות',
            },
            xSeconds: { one: 'שנייה', two: 'שתי שניות', other: '{{count}} שניות' },
            halfAMinute: 'חצי דקה',
            lessThanXMinutes: {
              one: 'פחות מדקה',
              two: 'פחות משתי דקות',
              other: 'פחות מ־{{count}} דקות',
            },
            xMinutes: { one: 'דקה', two: 'שתי דקות', other: '{{count}} דקות' },
            aboutXHours: { one: 'כשעה', two: 'כשעתיים', other: 'כ־{{count}} שעות' },
            xHours: { one: 'שעה', two: 'שעתיים', other: '{{count}} שעות' },
            xDays: { one: 'יום', two: 'יומיים', other: '{{count}} ימים' },
            aboutXWeeks: { one: 'כשבוע', two: 'כשבועיים', other: 'כ־{{count}} שבועות' },
            xWeeks: { one: 'שבוע', two: 'שבועיים', other: '{{count}} שבועות' },
            aboutXMonths: { one: 'כחודש', two: 'כחודשיים', other: 'כ־{{count}} חודשים' },
            xMonths: { one: 'חודש', two: 'חודשיים', other: '{{count}} חודשים' },
            aboutXYears: { one: 'כשנה', two: 'כשנתיים', other: 'כ־{{count}} שנים' },
            xYears: { one: 'שנה', two: 'שנתיים', other: '{{count}} שנים' },
            overXYears: { one: 'יותר משנה', two: 'יותר משנתיים', other: 'יותר מ־{{count}} שנים' },
            almostXYears: { one: 'כמעט שנה', two: 'כמעט שנתיים', other: 'כמעט {{count}} שנים' },
          },
          _default = function formatDistance(token, count, options) {
            if ('xDays' === token && null != options && options.addSuffix && count <= 2)
              return options.comparison && options.comparison > 0
                ? 1 === count
                  ? 'מחר'
                  : 'מחרתיים'
                : 1 === count
                  ? 'אתמול'
                  : 'שלשום';
            var result,
              tokenValue = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : 2 === count
                      ? tokenValue.two
                      : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'בעוד ' + result
                  : 'לפני ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
