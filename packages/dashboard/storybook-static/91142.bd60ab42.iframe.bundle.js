'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [91142],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/es/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'el' eeee 'pasado a la' p",
            yesterday: "'ayer a la' p",
            today: "'hoy a la' p",
            tomorrow: "'mañana a la' p",
            nextWeek: "eeee 'a la' p",
            other: 'P',
          },
          formatRelativeLocalePlural = {
            lastWeek: "'el' eeee 'pasado a las' p",
            yesterday: "'ayer a las' p",
            today: "'hoy a las' p",
            tomorrow: "'mañana a las' p",
            nextWeek: "eeee 'a las' p",
            other: 'P',
          },
          _default = function formatRelative(token, date, _baseDate, _options) {
            return 1 !== date.getUTCHours()
              ? formatRelativeLocalePlural[token]
              : formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
