'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [73867],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'أقل من ثانية',
              two: 'أقل من زوز ثواني',
              threeToTen: 'أقل من {{count}} ثواني',
              other: 'أقل من {{count}} ثانية',
            },
            xSeconds: {
              one: 'ثانية',
              two: 'زوز ثواني',
              threeToTen: '{{count}} ثواني',
              other: '{{count}} ثانية',
            },
            halfAMinute: 'نص دقيقة',
            lessThanXMinutes: {
              one: 'أقل من دقيقة',
              two: 'أقل من دقيقتين',
              threeToTen: 'أقل من {{count}} دقايق',
              other: 'أقل من {{count}} دقيقة',
            },
            xMinutes: {
              one: 'دقيقة',
              two: 'دقيقتين',
              threeToTen: '{{count}} دقايق',
              other: '{{count}} دقيقة',
            },
            aboutXHours: {
              one: 'ساعة تقريب',
              two: 'ساعتين تقريب',
              threeToTen: '{{count}} سوايع تقريب',
              other: '{{count}} ساعة تقريب',
            },
            xHours: {
              one: 'ساعة',
              two: 'ساعتين',
              threeToTen: '{{count}} سوايع',
              other: '{{count}} ساعة',
            },
            xDays: {
              one: 'نهار',
              two: 'نهارين',
              threeToTen: '{{count}} أيام',
              other: '{{count}} يوم',
            },
            aboutXWeeks: {
              one: 'جمعة تقريب',
              two: 'جمعتين تقريب',
              threeToTen: '{{count}} جماع تقريب',
              other: '{{count}} جمعة تقريب',
            },
            xWeeks: {
              one: 'جمعة',
              two: 'جمعتين',
              threeToTen: '{{count}} جماع',
              other: '{{count}} جمعة',
            },
            aboutXMonths: {
              one: 'شهر تقريب',
              two: 'شهرين تقريب',
              threeToTen: '{{count}} أشهرة تقريب',
              other: '{{count}} شهر تقريب',
            },
            xMonths: {
              one: 'شهر',
              two: 'شهرين',
              threeToTen: '{{count}} أشهرة',
              other: '{{count}} شهر',
            },
            aboutXYears: {
              one: 'عام تقريب',
              two: 'عامين تقريب',
              threeToTen: '{{count}} أعوام تقريب',
              other: '{{count}} عام تقريب',
            },
            xYears: {
              one: 'عام',
              two: 'عامين',
              threeToTen: '{{count}} أعوام',
              other: '{{count}} عام',
            },
            overXYears: {
              one: 'أكثر من عام',
              two: 'أكثر من عامين',
              threeToTen: 'أكثر من {{count}} أعوام',
              other: 'أكثر من {{count}} عام',
            },
            almostXYears: {
              one: 'عام تقريب',
              two: 'عامين تقريب',
              threeToTen: '{{count}} أعوام تقريب',
              other: '{{count}} عام تقريب',
            },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              usageGroup = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof usageGroup
                  ? usageGroup
                  : 1 === count
                    ? usageGroup.one
                    : 2 === count
                      ? usageGroup.two
                      : count <= 10
                        ? usageGroup.threeToTen.replace('{{count}}', String(count))
                        : usageGroup.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'في ' + result
                  : 'عندو ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
