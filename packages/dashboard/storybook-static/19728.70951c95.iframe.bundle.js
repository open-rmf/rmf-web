'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [19728],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: '1秒未満',
              other: '{{count}}秒未満',
              oneWithSuffix: '約1秒',
              otherWithSuffix: '約{{count}}秒',
            },
            xSeconds: { one: '1秒', other: '{{count}}秒' },
            halfAMinute: '30秒',
            lessThanXMinutes: {
              one: '1分未満',
              other: '{{count}}分未満',
              oneWithSuffix: '約1分',
              otherWithSuffix: '約{{count}}分',
            },
            xMinutes: { one: '1分', other: '{{count}}分' },
            aboutXHours: { one: '約1時間', other: '約{{count}}時間' },
            xHours: { one: '1時間', other: '{{count}}時間' },
            xDays: { one: '1日', other: '{{count}}日' },
            aboutXWeeks: { one: '約1週間', other: '約{{count}}週間' },
            xWeeks: { one: '1週間', other: '{{count}}週間' },
            aboutXMonths: { one: '約1か月', other: '約{{count}}か月' },
            xMonths: { one: '1か月', other: '{{count}}か月' },
            aboutXYears: { one: '約1年', other: '約{{count}}年' },
            xYears: { one: '1年', other: '{{count}}年' },
            overXYears: { one: '1年以上', other: '{{count}}年以上' },
            almostXYears: { one: '1年近く', other: '{{count}}年近く' },
          },
          _default = function formatDistance(token, count, options) {
            var result;
            options = options || {};
            var tokenValue = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? options.addSuffix && tokenValue.oneWithSuffix
                      ? tokenValue.oneWithSuffix
                      : tokenValue.one
                    : options.addSuffix && tokenValue.otherWithSuffix
                      ? tokenValue.otherWithSuffix.replace('{{count}}', String(count))
                      : tokenValue.other.replace('{{count}}', String(count))),
              options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? result + '後'
                  : result + '前'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
