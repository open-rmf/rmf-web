'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [62161],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/nn/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'mindre enn eitt sekund',
              other: 'mindre enn {{count}} sekund',
            },
            xSeconds: { one: 'eitt sekund', other: '{{count}} sekund' },
            halfAMinute: 'eit halvt minutt',
            lessThanXMinutes: {
              one: 'mindre enn eitt minutt',
              other: 'mindre enn {{count}} minutt',
            },
            xMinutes: { one: 'eitt minutt', other: '{{count}} minutt' },
            aboutXHours: { one: 'omtrent ein time', other: 'omtrent {{count}} timar' },
            xHours: { one: 'ein time', other: '{{count}} timar' },
            xDays: { one: 'ein dag', other: '{{count}} dagar' },
            aboutXWeeks: { one: 'omtrent ei veke', other: 'omtrent {{count}} veker' },
            xWeeks: { one: 'ei veke', other: '{{count}} veker' },
            aboutXMonths: { one: 'omtrent ein månad', other: 'omtrent {{count}} månader' },
            xMonths: { one: 'ein månad', other: '{{count}} månader' },
            aboutXYears: { one: 'omtrent eitt år', other: 'omtrent {{count}} år' },
            xYears: { one: 'eitt år', other: '{{count}} år' },
            overXYears: { one: 'over eitt år', other: 'over {{count}} år' },
            almostXYears: { one: 'nesten eitt år', other: 'nesten {{count}} år' },
          },
          wordMapping = [
            'null',
            'ein',
            'to',
            'tre',
            'fire',
            'fem',
            'seks',
            'sju',
            'åtte',
            'ni',
            'ti',
            'elleve',
            'tolv',
          ],
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : options && options.onlyNumeric
                      ? tokenValue.other.replace('{{count}}', String(count))
                      : tokenValue.other.replace(
                          '{{count}}',
                          count < 13 ? wordMapping[count] : String(count),
                        )),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'om ' + result
                  : result + ' sidan'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
