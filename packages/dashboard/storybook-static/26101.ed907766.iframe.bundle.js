'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [26101],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uz/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'oldingi' eeee p 'da'",
            yesterday: "'kecha' p 'da'",
            today: "'bugun' p 'da'",
            tomorrow: "'ertaga' p 'da'",
            nextWeek: "eeee p 'da'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
