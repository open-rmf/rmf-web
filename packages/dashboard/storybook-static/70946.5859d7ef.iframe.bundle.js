'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [70946],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/eo/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'pasinta' eeee 'je' p",
            yesterday: "'hieraŭ je' p",
            today: "'hodiaŭ je' p",
            tomorrow: "'morgaŭ je' p",
            nextWeek: "eeee 'je' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
