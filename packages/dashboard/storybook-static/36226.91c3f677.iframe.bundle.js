'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [36226],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: "'ថ្ងៃ'eeee'ស​ប្តា​ហ៍​មុនម៉ោង' p",
            yesterday: "'ម្សិលមិញនៅម៉ោង' p",
            today: "'ថ្ងៃនេះម៉ោង' p",
            tomorrow: "'ថ្ងៃស្អែកម៉ោង' p",
            nextWeek: "'ថ្ងៃ'eeee'ស​ប្តា​ហ៍​ក្រោយម៉ោង' p",
            other: 'P',
          },
          _default = function formatRelative(token, _date, _baseDate, _options) {
            return formatRelativeLocale[token];
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
