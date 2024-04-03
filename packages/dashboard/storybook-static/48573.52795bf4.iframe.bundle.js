'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [48573],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ja-Hira/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: '1びょうみまん',
              other: '{{count}}びょうみまん',
              oneWithSuffix: 'やく1びょう',
              otherWithSuffix: 'やく{{count}}びょう',
            },
            xSeconds: { one: '1びょう', other: '{{count}}びょう' },
            halfAMinute: '30びょう',
            lessThanXMinutes: {
              one: '1ぷんみまん',
              other: '{{count}}ふんみまん',
              oneWithSuffix: 'やく1ぷん',
              otherWithSuffix: 'やく{{count}}ふん',
            },
            xMinutes: { one: '1ぷん', other: '{{count}}ふん' },
            aboutXHours: { one: 'やく1じかん', other: 'やく{{count}}じかん' },
            xHours: { one: '1じかん', other: '{{count}}じかん' },
            xDays: { one: '1にち', other: '{{count}}にち' },
            aboutXWeeks: { one: 'やく1しゅうかん', other: 'やく{{count}}しゅうかん' },
            xWeeks: { one: '1しゅうかん', other: '{{count}}しゅうかん' },
            aboutXMonths: { one: 'やく1かげつ', other: 'やく{{count}}かげつ' },
            xMonths: { one: '1かげつ', other: '{{count}}かげつ' },
            aboutXYears: { one: 'やく1ねん', other: 'やく{{count}}ねん' },
            xYears: { one: '1ねん', other: '{{count}}ねん' },
            overXYears: { one: '1ねんいじょう', other: '{{count}}ねんいじょう' },
            almostXYears: { one: '1ねんちかく', other: '{{count}}ねんちかく' },
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
                  ? result + 'あと'
                  : result + 'まえ'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
