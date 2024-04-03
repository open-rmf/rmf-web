'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [97085],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/af/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'verlede' eeee 'om' p",
            yesterday: "'gister om' p",
            today: "'vandag om' p",
            tomorrow: "'môre om' p",
            nextWeek: "eeee 'om' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
