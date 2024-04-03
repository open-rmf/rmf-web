'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [72537],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: "menys d'un segon",
              eleven: "menys d'onze segons",
              other: 'menys de {{count}} segons',
            },
            xSeconds: { one: '1 segon', other: '{{count}} segons' },
            halfAMinute: 'mig minut',
            lessThanXMinutes: {
              one: "menys d'un minut",
              eleven: "menys d'onze minuts",
              other: 'menys de {{count}} minuts',
            },
            xMinutes: { one: '1 minut', other: '{{count}} minuts' },
            aboutXHours: {
              one: 'aproximadament una hora',
              other: 'aproximadament {{count}} hores',
            },
            xHours: { one: '1 hora', other: '{{count}} hores' },
            xDays: { one: '1 dia', other: '{{count}} dies' },
            aboutXWeeks: {
              one: 'aproximadament una setmana',
              other: 'aproximadament {{count}} setmanes',
            },
            xWeeks: { one: '1 setmana', other: '{{count}} setmanes' },
            aboutXMonths: { one: 'aproximadament un mes', other: 'aproximadament {{count}} mesos' },
            xMonths: { one: '1 mes', other: '{{count}} mesos' },
            aboutXYears: { one: 'aproximadament un any', other: 'aproximadament {{count}} anys' },
            xYears: { one: '1 any', other: '{{count}} anys' },
            overXYears: {
              one: "més d'un any",
              eleven: "més d'onze anys",
              other: 'més de {{count}} anys',
            },
            almostXYears: { one: 'gairebé un any', other: 'gairebé {{count}} anys' },
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
                    : 11 === count && tokenValue.eleven
                      ? tokenValue.eleven
                      : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'en ' + result
                  : 'fa ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
