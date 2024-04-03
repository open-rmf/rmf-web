'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [35793],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-CA/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'less than a second', other: 'less than {{count}} seconds' },
            xSeconds: { one: 'a second', other: '{{count}} seconds' },
            halfAMinute: 'half a minute',
            lessThanXMinutes: { one: 'less than a minute', other: 'less than {{count}} minutes' },
            xMinutes: { one: 'a minute', other: '{{count}} minutes' },
            aboutXHours: { one: 'about an hour', other: 'about {{count}} hours' },
            xHours: { one: 'an hour', other: '{{count}} hours' },
            xDays: { one: 'a day', other: '{{count}} days' },
            aboutXWeeks: { one: 'about a week', other: 'about {{count}} weeks' },
            xWeeks: { one: 'a week', other: '{{count}} weeks' },
            aboutXMonths: { one: 'about a month', other: 'about {{count}} months' },
            xMonths: { one: 'a month', other: '{{count}} months' },
            aboutXYears: { one: 'about a year', other: 'about {{count}} years' },
            xYears: { one: 'a year', other: '{{count}} years' },
            overXYears: { one: 'over a year', other: 'over {{count}} years' },
            almostXYears: { one: 'almost a year', other: 'almost {{count}} years' },
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
                  ? 'in ' + result
                  : result + ' ago'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
