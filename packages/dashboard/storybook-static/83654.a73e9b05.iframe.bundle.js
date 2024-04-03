'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [83654],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: "minder as 'n sekonde",
              other: 'minder as {{count}} sekondes',
            },
            xSeconds: { one: '1 sekonde', other: '{{count}} sekondes' },
            halfAMinute: "'n halwe minuut",
            lessThanXMinutes: { one: "minder as 'n minuut", other: 'minder as {{count}} minute' },
            xMinutes: { one: "'n minuut", other: '{{count}} minute' },
            aboutXHours: { one: 'ongeveer 1 uur', other: 'ongeveer {{count}} ure' },
            xHours: { one: '1 uur', other: '{{count}} ure' },
            xDays: { one: '1 dag', other: '{{count}} dae' },
            aboutXWeeks: { one: 'ongeveer 1 week', other: 'ongeveer {{count}} weke' },
            xWeeks: { one: '1 week', other: '{{count}} weke' },
            aboutXMonths: { one: 'ongeveer 1 maand', other: 'ongeveer {{count}} maande' },
            xMonths: { one: '1 maand', other: '{{count}} maande' },
            aboutXYears: { one: 'ongeveer 1 jaar', other: 'ongeveer {{count}} jaar' },
            xYears: { one: '1 jaar', other: '{{count}} jaar' },
            overXYears: { one: 'meer as 1 jaar', other: 'meer as {{count}} jaar' },
            almostXYears: { one: 'byna 1 jaar', other: 'byna {{count}} jaar' },
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
                  ? 'oor ' + result
                  : result + ' gelede'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
