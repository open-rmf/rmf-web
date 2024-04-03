'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [38509],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hy/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'նախորդ' eeee p'֊ին'",
            yesterday: "'երեկ' p'֊ին'",
            today: "'այսօր' p'֊ին'",
            tomorrow: "'վաղը' p'֊ին'",
            nextWeek: "'հաջորդ' eeee p'֊ին'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
