'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [34213],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'malpli ol sekundo', other: 'malpli ol {{count}} sekundoj' },
            xSeconds: { one: '1 sekundo', other: '{{count}} sekundoj' },
            halfAMinute: 'duonminuto',
            lessThanXMinutes: { one: 'malpli ol minuto', other: 'malpli ol {{count}} minutoj' },
            xMinutes: { one: '1 minuto', other: '{{count}} minutoj' },
            aboutXHours: { one: 'proksimume 1 horo', other: 'proksimume {{count}} horoj' },
            xHours: { one: '1 horo', other: '{{count}} horoj' },
            xDays: { one: '1 tago', other: '{{count}} tagoj' },
            aboutXMonths: { one: 'proksimume 1 monato', other: 'proksimume {{count}} monatoj' },
            xWeeks: { one: '1 semajno', other: '{{count}} semajnoj' },
            aboutXWeeks: { one: 'proksimume 1 semajno', other: 'proksimume {{count}} semajnoj' },
            xMonths: { one: '1 monato', other: '{{count}} monatoj' },
            aboutXYears: { one: 'proksimume 1 jaro', other: 'proksimume {{count}} jaroj' },
            xYears: { one: '1 jaro', other: '{{count}} jaroj' },
            overXYears: { one: 'pli ol 1 jaro', other: 'pli ol {{count}} jaroj' },
            almostXYears: { one: 'preskaŭ 1 jaro', other: 'preskaŭ {{count}} jaroj' },
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
                ? null != options && options.comparison && options.comparison > 0
                  ? 'post ' + result
                  : 'antaŭ ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
