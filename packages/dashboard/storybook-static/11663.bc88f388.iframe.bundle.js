'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [11663],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/et/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'eelmine' eeee 'kell' p",
            yesterday: "'eile kell' p",
            today: "'täna kell' p",
            tomorrow: "'homme kell' p",
            nextWeek: "'järgmine' eeee 'kell' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
