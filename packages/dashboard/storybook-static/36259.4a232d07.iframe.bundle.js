'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [36259],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-MA/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'أقل من ثانية واحدة',
              two: 'أقل من ثانتين',
              threeToTen: 'أقل من {{count}} ثواني',
              other: 'أقل من {{count}} ثانية',
            },
            xSeconds: {
              one: 'ثانية واحدة',
              two: 'ثانتين',
              threeToTen: '{{count}} ثواني',
              other: '{{count}} ثانية',
            },
            halfAMinute: 'نصف دقيقة',
            lessThanXMinutes: {
              one: 'أقل من دقيقة',
              two: 'أقل من دقيقتين',
              threeToTen: 'أقل من {{count}} دقائق',
              other: 'أقل من {{count}} دقيقة',
            },
            xMinutes: {
              one: 'دقيقة واحدة',
              two: 'دقيقتين',
              threeToTen: '{{count}} دقائق',
              other: '{{count}} دقيقة',
            },
            aboutXHours: {
              one: 'ساعة واحدة تقريباً',
              two: 'ساعتين تقريباً',
              threeToTen: '{{count}} ساعات تقريباً',
              other: '{{count}} ساعة تقريباً',
            },
            xHours: {
              one: 'ساعة واحدة',
              two: 'ساعتين',
              threeToTen: '{{count}} ساعات',
              other: '{{count}} ساعة',
            },
            xDays: {
              one: 'يوم واحد',
              two: 'يومين',
              threeToTen: '{{count}} أيام',
              other: '{{count}} يوم',
            },
            aboutXWeeks: {
              one: 'أسبوع واحد تقريباً',
              two: 'أسبوعين تقريباً',
              threeToTen: '{{count}} أسابيع تقريباً',
              other: '{{count}} أسبوع تقريباً',
            },
            xWeeks: {
              one: 'أسبوع واحد',
              two: 'أسبوعين',
              threeToTen: '{{count}} أسابيع',
              other: '{{count}} أسبوع',
            },
            aboutXMonths: {
              one: 'شهر واحد تقريباً',
              two: 'شهرين تقريباً',
              threeToTen: '{{count}} أشهر تقريباً',
              other: '{{count}} شهر تقريباً',
            },
            xMonths: {
              one: 'شهر واحد',
              two: 'شهرين',
              threeToTen: '{{count}} أشهر',
              other: '{{count}} شهر',
            },
            aboutXYears: {
              one: 'عام واحد تقريباً',
              two: 'عامين تقريباً',
              threeToTen: '{{count}} أعوام تقريباً',
              other: '{{count}} عام تقريباً',
            },
            xYears: {
              one: 'عام واحد',
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
              one: 'عام واحد تقريباً',
              two: 'عامين تقريباً',
              threeToTen: '{{count}} أعوام تقريباً',
              other: '{{count}} عام تقريباً',
            },
          },
          _default = function formatDistance(token, count, options) {
            options = options || {};
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
              options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'في خلال ' + result
                  : 'منذ ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
