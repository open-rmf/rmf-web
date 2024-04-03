'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [80736],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/zh-TW/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'上個'eeee p",
            yesterday: "'昨天' p",
            today: "'今天' p",
            tomorrow: "'明天' p",
            nextWeek: "'下個'eeee p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
