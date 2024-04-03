'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [65021],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'น้อยกว่า 1 วินาที', other: 'น้อยกว่า {{count}} วินาที' },
            xSeconds: { one: '1 วินาที', other: '{{count}} วินาที' },
            halfAMinute: 'ครึ่งนาที',
            lessThanXMinutes: { one: 'น้อยกว่า 1 นาที', other: 'น้อยกว่า {{count}} นาที' },
            xMinutes: { one: '1 นาที', other: '{{count}} นาที' },
            aboutXHours: { one: 'ประมาณ 1 ชั่วโมง', other: 'ประมาณ {{count}} ชั่วโมง' },
            xHours: { one: '1 ชั่วโมง', other: '{{count}} ชั่วโมง' },
            xDays: { one: '1 วัน', other: '{{count}} วัน' },
            aboutXWeeks: { one: 'ประมาณ 1 สัปดาห์', other: 'ประมาณ {{count}} สัปดาห์' },
            xWeeks: { one: '1 สัปดาห์', other: '{{count}} สัปดาห์' },
            aboutXMonths: { one: 'ประมาณ 1 เดือน', other: 'ประมาณ {{count}} เดือน' },
            xMonths: { one: '1 เดือน', other: '{{count}} เดือน' },
            aboutXYears: { one: 'ประมาณ 1 ปี', other: 'ประมาณ {{count}} ปี' },
            xYears: { one: '1 ปี', other: '{{count}} ปี' },
            overXYears: { one: 'มากกว่า 1 ปี', other: 'มากกว่า {{count}} ปี' },
            almostXYears: { one: 'เกือบ 1 ปี', other: 'เกือบ {{count}} ปี' },
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
                  ? 'halfAMinute' === token
                    ? 'ใน' + result
                    : 'ใน ' + result
                  : result + 'ที่ผ่านมา'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
