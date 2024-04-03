'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [36256],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/tr/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'geçen hafta' eeee 'saat' p",
            yesterday: "'dün saat' p",
            today: "'bugün saat' p",
            tomorrow: "'yarın saat' p",
            nextWeek: "eeee 'saat' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
