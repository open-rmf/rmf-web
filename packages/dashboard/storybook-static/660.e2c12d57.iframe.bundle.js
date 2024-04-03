'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [660],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fy/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'minder as 1 sekonde', other: 'minder as {{count}} sekonden' },
            xSeconds: { one: '1 sekonde', other: '{{count}} sekonden' },
            halfAMinute: 'oardel minút',
            lessThanXMinutes: { one: 'minder as 1 minút', other: 'minder as {{count}} minuten' },
            xMinutes: { one: '1 minút', other: '{{count}} minuten' },
            aboutXHours: { one: 'sawat 1 oere', other: 'sawat {{count}} oere' },
            xHours: { one: '1 oere', other: '{{count}} oere' },
            xDays: { one: '1 dei', other: '{{count}} dagen' },
            aboutXWeeks: { one: 'sawat 1 wike', other: 'sawat {{count}} wiken' },
            xWeeks: { one: '1 wike', other: '{{count}} wiken' },
            aboutXMonths: { one: 'sawat 1 moanne', other: 'sawat {{count}} moannen' },
            xMonths: { one: '1 moanne', other: '{{count}} moannen' },
            aboutXYears: { one: 'sawat 1 jier', other: 'sawat {{count}} jier' },
            xYears: { one: '1 jier', other: '{{count}} jier' },
            overXYears: { one: 'mear as 1 jier', other: 'mear as {{count}}s jier' },
            almostXYears: { one: 'hast 1 jier', other: 'hast {{count}} jier' },
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
                  ? 'oer ' + result
                  : result + ' lyn'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
