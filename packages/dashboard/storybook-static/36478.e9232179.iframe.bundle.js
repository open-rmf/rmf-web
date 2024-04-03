'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [36478],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'menos dun segundo', other: 'menos de {{count}} segundos' },
            xSeconds: { one: '1 segundo', other: '{{count}} segundos' },
            halfAMinute: 'medio minuto',
            lessThanXMinutes: { one: 'menos dun minuto', other: 'menos de {{count}} minutos' },
            xMinutes: { one: '1 minuto', other: '{{count}} minutos' },
            aboutXHours: { one: 'arredor dunha hora', other: 'arredor de {{count}} horas' },
            xHours: { one: '1 hora', other: '{{count}} horas' },
            xDays: { one: '1 día', other: '{{count}} días' },
            aboutXWeeks: { one: 'arredor dunha semana', other: 'arredor de {{count}} semanas' },
            xWeeks: { one: '1 semana', other: '{{count}} semanas' },
            aboutXMonths: { one: 'arredor de 1 mes', other: 'arredor de {{count}} meses' },
            xMonths: { one: '1 mes', other: '{{count}} meses' },
            aboutXYears: { one: 'arredor dun ano', other: 'arredor de {{count}} anos' },
            xYears: { one: '1 ano', other: '{{count}} anos' },
            overXYears: { one: 'máis dun ano', other: 'máis de {{count}} anos' },
            almostXYears: { one: 'case un ano', other: 'case {{count}} anos' },
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
                  ? 'en ' + result
                  : 'hai ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
