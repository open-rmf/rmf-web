'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [50882],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/_lib/formatDistance/index.js':
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
              one: 'ثانية واحدة',
              two: 'ثانيتان',
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
              two: 'دقيقتان',
              threeToTen: '{{count}} دقائق',
              other: '{{count}} دقيقة',
            },
            aboutXHours: {
              one: 'ساعة واحدة تقريباً',
              two: 'ساعتين تقريبا',
              threeToTen: '{{count}} ساعات تقريباً',
              other: '{{count}} ساعة تقريباً',
            },
            xHours: {
              one: 'ساعة واحدة',
              two: 'ساعتان',
              threeToTen: '{{count}} ساعات',
              other: '{{count}} ساعة',
            },
            xDays: {
              one: 'يوم واحد',
              two: 'يومان',
              threeToTen: '{{count}} أيام',
              other: '{{count}} يوم',
            },
            aboutXWeeks: {
              one: 'أسبوع واحد تقريبا',
              two: 'أسبوعين تقريبا',
              threeToTen: '{{count}} أسابيع تقريبا',
              other: '{{count}} أسبوعا تقريبا',
            },
            xWeeks: {
              one: 'أسبوع واحد',
              two: 'أسبوعان',
              threeToTen: '{{count}} أسابيع',
              other: '{{count}} أسبوعا',
            },
            aboutXMonths: {
              one: 'شهر واحد تقريباً',
              two: 'شهرين تقريبا',
              threeToTen: '{{count}} أشهر تقريبا',
              other: '{{count}} شهرا تقريباً',
            },
            xMonths: {
              one: 'شهر واحد',
              two: 'شهران',
              threeToTen: '{{count}} أشهر',
              other: '{{count}} شهرا',
            },
            aboutXYears: {
              one: 'سنة واحدة تقريباً',
              two: 'سنتين تقريبا',
              threeToTen: '{{count}} سنوات تقريباً',
              other: '{{count}} سنة تقريباً',
            },
            xYears: {
              one: 'سنة واحد',
              two: 'سنتان',
              threeToTen: '{{count}} سنوات',
              other: '{{count}} سنة',
            },
            overXYears: {
              one: 'أكثر من سنة',
              two: 'أكثر من سنتين',
              threeToTen: 'أكثر من {{count}} سنوات',
              other: 'أكثر من {{count}} سنة',
            },
            almostXYears: {
              one: 'ما يقارب سنة واحدة',
              two: 'ما يقارب سنتين',
              threeToTen: 'ما يقارب {{count}} سنوات',
              other: 'ما يقارب {{count}} سنة',
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
                  ? 'خلال ' + result
                  : 'منذ ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
