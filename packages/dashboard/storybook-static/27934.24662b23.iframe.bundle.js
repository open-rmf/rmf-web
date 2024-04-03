'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [27934],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ca/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'el' eeee 'passat a la' LT",
            yesterday: "'ahir a la' p",
            today: "'avui a la' p",
            tomorrow: "'demà a la' p",
            nextWeek: "eeee 'a la' p",
            other: 'P',
          },
          formatRelativeLocalePlural = {
            lastWeek: "'el' eeee 'passat a les' p",
            yesterday: "'ahir a les' p",
            today: "'avui a les' p",
            tomorrow: "'demà a les' p",
            nextWeek: "eeee 'a les' p",
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
