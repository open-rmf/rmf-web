'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [65457],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'өнгөрсөн' eeee 'гарагийн' p 'цагт'",
            yesterday: "'өчигдөр' p 'цагт'",
            today: "'өнөөдөр' p 'цагт'",
            tomorrow: "'маргааш' p 'цагт'",
            nextWeek: "'ирэх' eeee 'гарагийн' p 'цагт'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
