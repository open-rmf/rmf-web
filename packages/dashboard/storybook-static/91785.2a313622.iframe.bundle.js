'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [91785],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/az/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'sonuncu' eeee p -'də'",
            yesterday: "'dünən' p -'də'",
            today: "'bugün' p -'də'",
            tomorrow: "'sabah' p -'də'",
            nextWeek: "eeee p -'də'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
