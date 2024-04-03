'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [10708],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'dưới 1 giây', other: 'dưới {{count}} giây' },
            xSeconds: { one: '1 giây', other: '{{count}} giây' },
            halfAMinute: 'nửa phút',
            lessThanXMinutes: { one: 'dưới 1 phút', other: 'dưới {{count}} phút' },
            xMinutes: { one: '1 phút', other: '{{count}} phút' },
            aboutXHours: { one: 'khoảng 1 giờ', other: 'khoảng {{count}} giờ' },
            xHours: { one: '1 giờ', other: '{{count}} giờ' },
            xDays: { one: '1 ngày', other: '{{count}} ngày' },
            aboutXWeeks: { one: 'khoảng 1 tuần', other: 'khoảng {{count}} tuần' },
            xWeeks: { one: '1 tuần', other: '{{count}} tuần' },
            aboutXMonths: { one: 'khoảng 1 tháng', other: 'khoảng {{count}} tháng' },
            xMonths: { one: '1 tháng', other: '{{count}} tháng' },
            aboutXYears: { one: 'khoảng 1 năm', other: 'khoảng {{count}} năm' },
            xYears: { one: '1 năm', other: '{{count}} năm' },
            overXYears: { one: 'hơn 1 năm', other: 'hơn {{count}} năm' },
            almostXYears: { one: 'gần 1 năm', other: 'gần {{count}} năm' },
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
                  ? result + ' nữa'
                  : result + ' trước'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
