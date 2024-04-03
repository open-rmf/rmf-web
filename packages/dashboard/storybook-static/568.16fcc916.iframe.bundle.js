'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [568],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eu/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'joan den' eeee, LT",
            yesterday: "'atzo,' p",
            today: "'gaur,' p",
            tomorrow: "'bihar,' p",
            nextWeek: 'eeee, p',
            other: 'P',
          },
          formatRelativeLocalePlural = {
            lastWeek: "'joan den' eeee, p",
            yesterday: "'atzo,' p",
            today: "'gaur,' p",
            tomorrow: "'bihar,' p",
            nextWeek: 'eeee, p',
            other: 'P',
          },
          _default = function formatRelative(token, date) {
            return 1 !== date.getUTCHours()
              ? formatRelativeLocalePlural[token]
              : formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
