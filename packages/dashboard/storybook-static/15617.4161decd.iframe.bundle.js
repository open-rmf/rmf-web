'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [15617],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gu/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'હમણાં', other: '​આશરે {{count}} સેકંડ' },
            xSeconds: { one: '1 સેકંડ', other: '{{count}} સેકંડ' },
            halfAMinute: 'અડધી મિનિટ',
            lessThanXMinutes: { one: 'આ મિનિટ', other: '​આશરે {{count}} મિનિટ' },
            xMinutes: { one: '1 મિનિટ', other: '{{count}} મિનિટ' },
            aboutXHours: { one: '​આશરે 1 કલાક', other: '​આશરે {{count}} કલાક' },
            xHours: { one: '1 કલાક', other: '{{count}} કલાક' },
            xDays: { one: '1 દિવસ', other: '{{count}} દિવસ' },
            aboutXWeeks: { one: 'આશરે 1 અઠવાડિયું', other: 'આશરે {{count}} અઠવાડિયા' },
            xWeeks: { one: '1 અઠવાડિયું', other: '{{count}} અઠવાડિયા' },
            aboutXMonths: { one: 'આશરે 1 મહિનો', other: 'આશરે {{count}} મહિના' },
            xMonths: { one: '1 મહિનો', other: '{{count}} મહિના' },
            aboutXYears: { one: 'આશરે 1 વર્ષ', other: 'આશરે {{count}} વર્ષ' },
            xYears: { one: '1 વર્ષ', other: '{{count}} વર્ષ' },
            overXYears: { one: '1 વર્ષથી વધુ', other: '{{count}} વર્ષથી વધુ' },
            almostXYears: { one: 'લગભગ 1 વર્ષ', other: 'લગભગ {{count}} વર્ષ' },
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
                  ? result + 'માં'
                  : result + ' પહેલાં'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
