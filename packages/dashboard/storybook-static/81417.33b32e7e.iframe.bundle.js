'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [81417],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ta/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'கடந்த' eeee p 'மணிக்கு'",
            yesterday: "'நேற்று ' p 'மணிக்கு'",
            today: "'இன்று ' p 'மணிக்கு'",
            tomorrow: "'நாளை ' p 'மணிக்கு'",
            nextWeek: "eeee p 'மணிக்கு'",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
