'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [17231],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: '少於 1 秒', other: '少於 {{count}} 秒' },
            xSeconds: { one: '1 秒', other: '{{count}} 秒' },
            halfAMinute: '半分鐘',
            lessThanXMinutes: { one: '少於 1 分鐘', other: '少於 {{count}} 分鐘' },
            xMinutes: { one: '1 分鐘', other: '{{count}} 分鐘' },
            xHours: { one: '1 小時', other: '{{count}} 小時' },
            aboutXHours: { one: '大約 1 小時', other: '大約 {{count}} 小時' },
            xDays: { one: '1 天', other: '{{count}} 天' },
            aboutXWeeks: { one: '大約 1 個星期', other: '大約 {{count}} 個星期' },
            xWeeks: { one: '1 個星期', other: '{{count}} 個星期' },
            aboutXMonths: { one: '大約 1 個月', other: '大約 {{count}} 個月' },
            xMonths: { one: '1 個月', other: '{{count}} 個月' },
            aboutXYears: { one: '大約 1 年', other: '大約 {{count}} 年' },
            xYears: { one: '1 年', other: '{{count}} 年' },
            overXYears: { one: '超過 1 年', other: '超過 {{count}} 年' },
            almostXYears: { one: '將近 1 年', other: '將近 {{count}} 年' },
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
                  ? result + '內'
                  : result + '前'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
