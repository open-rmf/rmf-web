'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [27194],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/th/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee'ที่แล้วเวลา' p",
            yesterday: "'เมื่อวานนี้เวลา' p",
            today: "'วันนี้เวลา' p",
            tomorrow: "'พรุ่งนี้เวลา' p",
            nextWeek: "eeee 'เวลา' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
