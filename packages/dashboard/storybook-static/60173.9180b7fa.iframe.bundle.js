'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [60173],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/gd/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'mu dheireadh' eeee 'aig' p",
            yesterday: "'an-dè aig' p",
            today: "'an-diugh aig' p",
            tomorrow: "'a-màireach aig' p",
            nextWeek: "eeee 'aig' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
