'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [32935],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date) {
              return 6 === date.getUTCDay()
                ? "'το προηγούμενο' eeee 'στις' p"
                : "'την προηγούμενη' eeee 'στις' p";
            },
            yesterday: "'χθες στις' p",
            today: "'σήμερα στις' p",
            tomorrow: "'αύριο στις' p",
            nextWeek: "eeee 'στις' p",
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
