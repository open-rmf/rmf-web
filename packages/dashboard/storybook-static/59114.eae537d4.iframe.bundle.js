'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [59114],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/bn/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'গত' eeee 'সময়' p",
            yesterday: "'গতকাল' 'সময়' p",
            today: "'আজ' 'সময়' p",
            tomorrow: "'আগামীকাল' 'সময়' p",
            nextWeek: "eeee 'সময়' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
