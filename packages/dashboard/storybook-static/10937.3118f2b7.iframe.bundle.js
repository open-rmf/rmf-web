'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [10937],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/he/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'שעבר בשעה' p",
            yesterday: "'אתמול בשעה' p",
            today: "'היום בשעה' p",
            tomorrow: "'מחר בשעה' p",
            nextWeek: "eeee 'בשעה' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
