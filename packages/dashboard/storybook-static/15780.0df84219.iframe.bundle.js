'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [15780],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ar-TN/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "eeee 'إلي فات مع' p",
            yesterday: "'البارح مع' p",
            today: "'اليوم مع' p",
            tomorrow: "'غدوة مع' p",
            nextWeek: "eeee 'الجمعة الجاية مع' p 'نهار'",
            other: 'P',
          },
          _default = function formatRelative(token) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
