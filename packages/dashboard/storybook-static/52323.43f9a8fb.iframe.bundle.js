'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [52323],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/id/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'lalu pukul' p",
            yesterday: "'Kemarin pukul' p",
            today: "'Hari ini pukul' p",
            tomorrow: "'Besok pukul' p",
            nextWeek: "eeee 'pukul' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
