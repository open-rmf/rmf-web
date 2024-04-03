'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [10253],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-EG/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'أقل من ثانية',
              two: 'أقل من ثانيتين',
              threeToTen: 'أقل من {{count}} ثواني',
              other: 'أقل من {{count}} ثانية',
            },
            xSeconds: {
              one: 'ثانية',
              two: 'ثانيتين',
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
              one: 'حوالي ساعة',
              two: 'حوالي ساعتين',
              threeToTen: 'حوالي {{count}} ساعات',
              other: 'حوالي {{count}} ساعة',
            },
            xHours: {
              one: 'ساعة',
              two: 'ساعتين',
              threeToTen: '{{count}} ساعات',
              other: '{{count}} ساعة',
            },
            xDays: {
              one: 'يوم',
              two: 'يومين',
              threeToTen: '{{count}} أيام',
              other: '{{count}} يوم',
            },
            aboutXWeeks: {
              one: 'حوالي أسبوع',
              two: 'حوالي أسبوعين',
              threeToTen: 'حوالي {{count}} أسابيع',
              other: 'حوالي {{count}} أسبوع',
            },
            xWeeks: {
              one: 'أسبوع',
              two: 'أسبوعين',
              threeToTen: '{{count}} أسابيع',
              other: '{{count}} أسبوع',
            },
            aboutXMonths: {
              one: 'حوالي شهر',
              two: 'حوالي شهرين',
              threeToTen: 'حوالي {{count}} أشهر',
              other: 'حوالي {{count}} شهر',
            },
            xMonths: {
              one: 'شهر',
              two: 'شهرين',
              threeToTen: '{{count}} أشهر',
              other: '{{count}} شهر',
            },
            aboutXYears: {
              one: 'حوالي سنة',
              two: 'حوالي سنتين',
              threeToTen: 'حوالي {{count}} سنين',
              other: 'حوالي {{count}} سنة',
            },
            xYears: {
              one: 'عام',
              two: 'عامين',
              threeToTen: '{{count}} أعوام',
              other: '{{count}} عام',
            },
            overXYears: {
              one: 'أكثر من سنة',
              two: 'أكثر من سنتين',
              threeToTen: 'أكثر من {{count}} سنين',
              other: 'أكثر من {{count}} سنة',
            },
            almostXYears: {
              one: 'عام تقريبًا',
              two: 'عامين تقريبًا',
              threeToTen: '{{count}} أعوام تقريبًا',
              other: '{{count}} عام تقريبًا',
            },
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
                    : 2 === count
                      ? tokenValue.two
                      : count <= 10
                        ? tokenValue.threeToTen.replace('{{count}}', String(count))
                        : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'في خلال '.concat(result)
                  : 'منذ '.concat(result)
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
