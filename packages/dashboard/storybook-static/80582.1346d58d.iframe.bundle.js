'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [80582],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/cy/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'diwethaf am' p",
            yesterday: "'ddoe am' p",
            today: "'heddiw am' p",
            tomorrow: "'yfory am' p",
            nextWeek: "eeee 'am' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
