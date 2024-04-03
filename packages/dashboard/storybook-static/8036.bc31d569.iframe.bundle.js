'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [8036],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'menos de um segundo', other: 'menos de {{count}} segundos' },
            xSeconds: { one: '1 segundo', other: '{{count}} segundos' },
            halfAMinute: 'meio minuto',
            lessThanXMinutes: { one: 'menos de um minuto', other: 'menos de {{count}} minutos' },
            xMinutes: { one: '1 minuto', other: '{{count}} minutos' },
            aboutXHours: { one: 'cerca de 1 hora', other: 'cerca de {{count}} horas' },
            xHours: { one: '1 hora', other: '{{count}} horas' },
            xDays: { one: '1 dia', other: '{{count}} dias' },
            aboutXWeeks: { one: 'cerca de 1 semana', other: 'cerca de {{count}} semanas' },
            xWeeks: { one: '1 semana', other: '{{count}} semanas' },
            aboutXMonths: { one: 'cerca de 1 mês', other: 'cerca de {{count}} meses' },
            xMonths: { one: '1 mês', other: '{{count}} meses' },
            aboutXYears: { one: 'cerca de 1 ano', other: 'cerca de {{count}} anos' },
            xYears: { one: '1 ano', other: '{{count}} anos' },
            overXYears: { one: 'mais de 1 ano', other: 'mais de {{count}} anos' },
            almostXYears: { one: 'quase 1 ano', other: 'quase {{count}} anos' },
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
                  ? 'em ' + result
                  : 'há ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
