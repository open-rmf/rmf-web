'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [23513],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sq/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'më pak se një sekondë',
              other: 'më pak se {{count}} sekonda',
            },
            xSeconds: { one: '1 sekondë', other: '{{count}} sekonda' },
            halfAMinute: 'gjysëm minuti',
            lessThanXMinutes: { one: 'më pak se një minute', other: 'më pak se {{count}} minuta' },
            xMinutes: { one: '1 minutë', other: '{{count}} minuta' },
            aboutXHours: { one: 'rreth 1 orë', other: 'rreth {{count}} orë' },
            xHours: { one: '1 orë', other: '{{count}} orë' },
            xDays: { one: '1 ditë', other: '{{count}} ditë' },
            aboutXWeeks: { one: 'rreth 1 javë', other: 'rreth {{count}} javë' },
            xWeeks: { one: '1 javë', other: '{{count}} javë' },
            aboutXMonths: { one: 'rreth 1 muaj', other: 'rreth {{count}} muaj' },
            xMonths: { one: '1 muaj', other: '{{count}} muaj' },
            aboutXYears: { one: 'rreth 1 vit', other: 'rreth {{count}} vite' },
            xYears: { one: '1 vit', other: '{{count}} vite' },
            overXYears: { one: 'mbi 1 vit', other: 'mbi {{count}} vite' },
            almostXYears: { one: 'pothuajse 1 vit', other: 'pothuajse {{count}} vite' },
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
                  ? 'në ' + result
                  : result + ' më parë'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
