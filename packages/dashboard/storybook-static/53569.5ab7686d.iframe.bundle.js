'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [53569],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'الماضي عند الساعة' p",
            yesterday: "'الأمس عند الساعة' p",
            today: "'اليوم عند الساعة' p",
            tomorrow: "'غدا عند الساعة' p",
            nextWeek: "eeee 'القادم عند الساعة' p",
            other: 'P',
          },
          _default = function formatRelative(token) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
