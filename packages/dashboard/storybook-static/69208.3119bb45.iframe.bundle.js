'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [69208],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr-CH/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'la semaine dernière à' p",
            yesterday: "'hier à' p",
            today: "'aujourd’hui à' p",
            tomorrow: "'demain à' p'",
            nextWeek: "eeee 'la semaine prochaine à' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
