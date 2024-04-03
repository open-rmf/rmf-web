'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [82742],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'nas lugha na diog', other: 'nas lugha na {{count}} diogan' },
            xSeconds: {
              one: '1 diog',
              two: '2 dhiog',
              twenty: '20 diog',
              other: '{{count}} diogan',
            },
            halfAMinute: 'leth mhionaid',
            lessThanXMinutes: {
              one: 'nas lugha na mionaid',
              other: 'nas lugha na {{count}} mionaidean',
            },
            xMinutes: {
              one: '1 mionaid',
              two: '2 mhionaid',
              twenty: '20 mionaid',
              other: '{{count}} mionaidean',
            },
            aboutXHours: { one: 'mu uair de thìde', other: 'mu {{count}} uairean de thìde' },
            xHours: {
              one: '1 uair de thìde',
              two: '2 uair de thìde',
              twenty: '20 uair de thìde',
              other: '{{count}} uairean de thìde',
            },
            xDays: { one: '1 là', other: '{{count}} là' },
            aboutXWeeks: { one: 'mu 1 seachdain', other: 'mu {{count}} seachdainean' },
            xWeeks: { one: '1 seachdain', other: '{{count}} seachdainean' },
            aboutXMonths: { one: 'mu mhìos', other: 'mu {{count}} mìosan' },
            xMonths: { one: '1 mìos', other: '{{count}} mìosan' },
            aboutXYears: { one: 'mu bhliadhna', other: 'mu {{count}} bliadhnaichean' },
            xYears: { one: '1 bhliadhna', other: '{{count}} bliadhna' },
            overXYears: { one: 'còrr is bliadhna', other: 'còrr is {{count}} bliadhnaichean' },
            almostXYears: { one: 'cha mhòr bliadhna', other: 'cha mhòr {{count}} bliadhnaichean' },
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
                      : 20 === count && tokenValue.twenty
                        ? tokenValue.twenty
                        : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'ann an ' + result
                  : 'o chionn ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
