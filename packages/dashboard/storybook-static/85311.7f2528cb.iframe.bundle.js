'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [85311],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ko/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: '1초 미만', other: '{{count}}초 미만' },
            xSeconds: { one: '1초', other: '{{count}}초' },
            halfAMinute: '30초',
            lessThanXMinutes: { one: '1분 미만', other: '{{count}}분 미만' },
            xMinutes: { one: '1분', other: '{{count}}분' },
            aboutXHours: { one: '약 1시간', other: '약 {{count}}시간' },
            xHours: { one: '1시간', other: '{{count}}시간' },
            xDays: { one: '1일', other: '{{count}}일' },
            aboutXWeeks: { one: '약 1주', other: '약 {{count}}주' },
            xWeeks: { one: '1주', other: '{{count}}주' },
            aboutXMonths: { one: '약 1개월', other: '약 {{count}}개월' },
            xMonths: { one: '1개월', other: '{{count}}개월' },
            aboutXYears: { one: '약 1년', other: '약 {{count}}년' },
            xYears: { one: '1년', other: '{{count}}년' },
            overXYears: { one: '1년 이상', other: '{{count}}년 이상' },
            almostXYears: { one: '거의 1년', other: '거의 {{count}}년' },
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
                  ? result + ' 후'
                  : result + ' 전'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
