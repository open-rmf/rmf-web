'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [43593],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var accusativeWeekdays = [
          'vasárnap',
          'hétfőn',
          'kedden',
          'szerdán',
          'csütörtökön',
          'pénteken',
          'szombaton',
        ];
        function week(isFuture) {
          return function (date) {
            var weekday = accusativeWeekdays[date.getUTCDay()];
            return ''.concat(isFuture ? '' : "'múlt' ", "'").concat(weekday, "' p'-kor'");
          };
        }
        var formatRelativeLocale = {
            lastWeek: week(!1),
            yesterday: "'tegnap' p'-kor'",
            today: "'ma' p'-kor'",
            tomorrow: "'holnap' p'-kor'",
            nextWeek: week(!0),
            other: 'P',
          },
          _default = function formatRelative(token, date) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
