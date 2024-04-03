'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [79425],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mk/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'помалку од секунда', other: 'помалку од {{count}} секунди' },
            xSeconds: { one: '1 секунда', other: '{{count}} секунди' },
            halfAMinute: 'половина минута',
            lessThanXMinutes: { one: 'помалку од минута', other: 'помалку од {{count}} минути' },
            xMinutes: { one: '1 минута', other: '{{count}} минути' },
            aboutXHours: { one: 'околу 1 час', other: 'околу {{count}} часа' },
            xHours: { one: '1 час', other: '{{count}} часа' },
            xDays: { one: '1 ден', other: '{{count}} дена' },
            aboutXWeeks: { one: 'околу 1 недела', other: 'околу {{count}} месеци' },
            xWeeks: { one: '1 недела', other: '{{count}} недели' },
            aboutXMonths: { one: 'околу 1 месец', other: 'околу {{count}} недели' },
            xMonths: { one: '1 месец', other: '{{count}} месеци' },
            aboutXYears: { one: 'околу 1 година', other: 'околу {{count}} години' },
            xYears: { one: '1 година', other: '{{count}} години' },
            overXYears: { one: 'повеќе од 1 година', other: 'повеќе од {{count}} години' },
            almostXYears: { one: 'безмалку 1 година', other: 'безмалку {{count}} години' },
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
                  ? 'за ' + result
                  : 'пред ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
