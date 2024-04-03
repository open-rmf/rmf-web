'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [66423],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pt-BR/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date) {
              var weekday = date.getUTCDay();
              return "'" + (0 === weekday || 6 === weekday ? 'último' : 'última') + "' eeee 'às' p";
            },
            yesterday: "'ontem às' p",
            today: "'hoje às' p",
            tomorrow: "'amanhã às' p",
            nextWeek: "eeee 'às' p",
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
