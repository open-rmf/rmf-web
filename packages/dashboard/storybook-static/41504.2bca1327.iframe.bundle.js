'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [41504],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'passat a' p",
            yesterday: "'ièr a' p",
            today: "'uèi a' p",
            tomorrow: "'deman a' p",
            nextWeek: "eeee 'a' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
