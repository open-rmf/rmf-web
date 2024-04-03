'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [76257],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'menos de un segundo', other: 'menos de {{count}} segundos' },
            xSeconds: { one: '1 segundo', other: '{{count}} segundos' },
            halfAMinute: 'medio minuto',
            lessThanXMinutes: { one: 'menos de un minuto', other: 'menos de {{count}} minutos' },
            xMinutes: { one: '1 minuto', other: '{{count}} minutos' },
            aboutXHours: { one: 'alrededor de 1 hora', other: 'alrededor de {{count}} horas' },
            xHours: { one: '1 hora', other: '{{count}} horas' },
            xDays: { one: '1 día', other: '{{count}} días' },
            aboutXWeeks: { one: 'alrededor de 1 semana', other: 'alrededor de {{count}} semanas' },
            xWeeks: { one: '1 semana', other: '{{count}} semanas' },
            aboutXMonths: { one: 'alrededor de 1 mes', other: 'alrededor de {{count}} meses' },
            xMonths: { one: '1 mes', other: '{{count}} meses' },
            aboutXYears: { one: 'alrededor de 1 año', other: 'alrededor de {{count}} años' },
            xYears: { one: '1 año', other: '{{count}} años' },
            overXYears: { one: 'más de 1 año', other: 'más de {{count}} años' },
            almostXYears: { one: 'casi 1 año', other: 'casi {{count}} años' },
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
                    : tokenValue.other.replace('{{count}}', count.toString())),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'en ' + result
                  : 'hace ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
