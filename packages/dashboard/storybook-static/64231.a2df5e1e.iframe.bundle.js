'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [64231],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/vi/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'tuần trước vào lúc' p",
            yesterday: "'hôm qua vào lúc' p",
            today: "'hôm nay vào lúc' p",
            tomorrow: "'ngày mai vào lúc' p",
            nextWeek: "eeee 'tới vào lúc' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
