'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [96647],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fa-IR/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'گذشته در' p",
            yesterday: "'دیروز در' p",
            today: "'امروز در' p",
            tomorrow: "'فردا در' p",
            nextWeek: "eeee 'در' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
