'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [65004],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cs/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var accusativeWeekdays = [
            'neděli',
            'pondělí',
            'úterý',
            'středu',
            'čtvrtek',
            'pátek',
            'sobotu',
          ],
          formatRelativeLocale = {
            lastWeek: "'poslední' eeee 've' p",
            yesterday: "'včera v' p",
            today: "'dnes v' p",
            tomorrow: "'zítra v' p",
            nextWeek: function nextWeek(date) {
              var day = date.getUTCDay();
              return "'v " + accusativeWeekdays[day] + " o' p";
            },
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
