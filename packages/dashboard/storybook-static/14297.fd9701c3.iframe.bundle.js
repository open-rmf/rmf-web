'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [14297],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/da/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'sidste' eeee 'kl.' p",
            yesterday: "'i går kl.' p",
            today: "'i dag kl.' p",
            tomorrow: "'i morgen kl.' p",
            nextWeek: "'på' eeee 'kl.' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
