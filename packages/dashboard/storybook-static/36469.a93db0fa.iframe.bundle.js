'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [36469],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bs/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date) {
              switch (date.getUTCDay()) {
                case 0:
                  return "'prošle nedjelje u' p";
                case 3:
                  return "'prošle srijede u' p";
                case 6:
                  return "'prošle subote u' p";
                default:
                  return "'prošli' EEEE 'u' p";
              }
            },
            yesterday: "'juče u' p",
            today: "'danas u' p",
            tomorrow: "'sutra u' p",
            nextWeek: function nextWeek(date) {
              switch (date.getUTCDay()) {
                case 0:
                  return "'sljedeće nedjelje u' p";
                case 3:
                  return "'sljedeću srijedu u' p";
                case 6:
                  return "'sljedeću subotu u' p";
                default:
                  return "'sljedeći' EEEE 'u' p";
              }
            },
            other: 'P',
          },
          _default = function formatRelative(token, date, _baseDate, _options) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
