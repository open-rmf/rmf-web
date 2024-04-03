'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [9493],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gl/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'o' eeee 'pasado á' LT",
            yesterday: "'onte á' p",
            today: "'hoxe á' p",
            tomorrow: "'mañá á' p",
            nextWeek: "eeee 'á' p",
            other: 'P',
          },
          formatRelativeLocalePlural = {
            lastWeek: "'o' eeee 'pasado ás' p",
            yesterday: "'onte ás' p",
            today: "'hoxe ás' p",
            tomorrow: "'mañá ás' p",
            nextWeek: "eeee 'ás' p",
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
