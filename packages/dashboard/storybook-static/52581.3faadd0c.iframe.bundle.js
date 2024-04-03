'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [52581],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'letzten' eeee 'um' p",
            yesterday: "'gestern um' p",
            today: "'heute um' p",
            tomorrow: "'morgen um' p",
            nextWeek: "eeee 'um' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
