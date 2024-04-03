'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [82868],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'کمتر از یک ثانیه', other: 'کمتر از {{count}} ثانیه' },
            xSeconds: { one: '1 ثانیه', other: '{{count}} ثانیه' },
            halfAMinute: 'نیم دقیقه',
            lessThanXMinutes: { one: 'کمتر از یک دقیقه', other: 'کمتر از {{count}} دقیقه' },
            xMinutes: { one: '1 دقیقه', other: '{{count}} دقیقه' },
            aboutXHours: { one: 'حدود 1 ساعت', other: 'حدود {{count}} ساعت' },
            xHours: { one: '1 ساعت', other: '{{count}} ساعت' },
            xDays: { one: '1 روز', other: '{{count}} روز' },
            aboutXWeeks: { one: 'حدود 1 هفته', other: 'حدود {{count}} هفته' },
            xWeeks: { one: '1 هفته', other: '{{count}} هفته' },
            aboutXMonths: { one: 'حدود 1 ماه', other: 'حدود {{count}} ماه' },
            xMonths: { one: '1 ماه', other: '{{count}} ماه' },
            aboutXYears: { one: 'حدود 1 سال', other: 'حدود {{count}} سال' },
            xYears: { one: '1 سال', other: '{{count}} سال' },
            overXYears: { one: 'بیشتر از 1 سال', other: 'بیشتر از {{count}} سال' },
            almostXYears: { one: 'نزدیک 1 سال', other: 'نزدیک {{count}} سال' },
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
                  ? 'در ' + result
                  : result + ' قبل'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
