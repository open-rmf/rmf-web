'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [36103],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mt/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'li għadda' 'fil-'p",
            yesterday: "'Il-bieraħ fil-'p",
            today: "'Illum fil-'p",
            tomorrow: "'Għada fil-'p",
            nextWeek: "eeee 'fil-'p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
