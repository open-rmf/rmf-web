'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [41993],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/en-US/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'less than a second', other: 'less than {{count}} seconds' },
            xSeconds: { one: '1 second', other: '{{count}} seconds' },
            halfAMinute: 'half a minute',
            lessThanXMinutes: { one: 'less than a minute', other: 'less than {{count}} minutes' },
            xMinutes: { one: '1 minute', other: '{{count}} minutes' },
            aboutXHours: { one: 'about 1 hour', other: 'about {{count}} hours' },
            xHours: { one: '1 hour', other: '{{count}} hours' },
            xDays: { one: '1 day', other: '{{count}} days' },
            aboutXWeeks: { one: 'about 1 week', other: 'about {{count}} weeks' },
            xWeeks: { one: '1 week', other: '{{count}} weeks' },
            aboutXMonths: { one: 'about 1 month', other: 'about {{count}} months' },
            xMonths: { one: '1 month', other: '{{count}} months' },
            aboutXYears: { one: 'about 1 year', other: 'about {{count}} years' },
            xYears: { one: '1 year', other: '{{count}} years' },
            overXYears: { one: 'over 1 year', other: 'over {{count}} years' },
            almostXYears: { one: 'almost 1 year', other: 'almost {{count}} years' },
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
