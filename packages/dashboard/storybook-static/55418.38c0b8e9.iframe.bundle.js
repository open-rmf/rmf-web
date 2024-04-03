'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [55418],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bg/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'по-малко от секунда',
              other: 'по-малко от {{count}} секунди',
            },
            xSeconds: { one: '1 секунда', other: '{{count}} секунди' },
            halfAMinute: 'половин минута',
            lessThanXMinutes: { one: 'по-малко от минута', other: 'по-малко от {{count}} минути' },
            xMinutes: { one: '1 минута', other: '{{count}} минути' },
            aboutXHours: { one: 'около час', other: 'около {{count}} часа' },
            xHours: { one: '1 час', other: '{{count}} часа' },
            xDays: { one: '1 ден', other: '{{count}} дни' },
            aboutXWeeks: { one: 'около седмица', other: 'около {{count}} седмици' },
            xWeeks: { one: '1 седмица', other: '{{count}} седмици' },
            aboutXMonths: { one: 'около месец', other: 'около {{count}} месеца' },
            xMonths: { one: '1 месец', other: '{{count}} месеца' },
            aboutXYears: { one: 'около година', other: 'около {{count}} години' },
            xYears: { one: '1 година', other: '{{count}} години' },
            overXYears: { one: 'над година', other: 'над {{count}} години' },
            almostXYears: { one: 'почти година', other: 'почти {{count}} години' },
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
                  ? 'след ' + result
                  : 'преди ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
