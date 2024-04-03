'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [12583],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/kn/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'ಕಳೆದ' eeee p 'ಕ್ಕೆ'",
            yesterday: "'ನಿನ್ನೆ' p 'ಕ್ಕೆ'",
            today: "'ಇಂದು' p 'ಕ್ಕೆ'",
            tomorrow: "'ನಾಳೆ' p 'ಕ್ಕೆ'",
            nextWeek: "eeee p 'ಕ್ಕೆ'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
