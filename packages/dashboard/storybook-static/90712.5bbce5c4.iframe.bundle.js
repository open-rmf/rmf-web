'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [90712],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/it/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'meno di un secondo', other: 'meno di {{count}} secondi' },
            xSeconds: { one: 'un secondo', other: '{{count}} secondi' },
            halfAMinute: 'alcuni secondi',
            lessThanXMinutes: { one: 'meno di un minuto', other: 'meno di {{count}} minuti' },
            xMinutes: { one: 'un minuto', other: '{{count}} minuti' },
            aboutXHours: { one: "circa un'ora", other: 'circa {{count}} ore' },
            xHours: { one: "un'ora", other: '{{count}} ore' },
            xDays: { one: 'un giorno', other: '{{count}} giorni' },
            aboutXWeeks: { one: 'circa una settimana', other: 'circa {{count}} settimane' },
            xWeeks: { one: 'una settimana', other: '{{count}} settimane' },
            aboutXMonths: { one: 'circa un mese', other: 'circa {{count}} mesi' },
            xMonths: { one: 'un mese', other: '{{count}} mesi' },
            aboutXYears: { one: 'circa un anno', other: 'circa {{count}} anni' },
            xYears: { one: 'un anno', other: '{{count}} anni' },
            overXYears: { one: 'più di un anno', other: 'più di {{count}} anni' },
            almostXYears: { one: 'quasi un anno', other: 'quasi {{count}} anni' },
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
                  ? 'tra ' + result
                  : result + ' fa'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
