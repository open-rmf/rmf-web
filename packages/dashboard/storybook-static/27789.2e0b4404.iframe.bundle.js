'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [27789],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ug/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'بىر سىكۇنت ئىچىدە', other: 'سىكۇنت ئىچىدە {{count}}' },
            xSeconds: { one: 'بىر سىكۇنت', other: 'سىكۇنت {{count}}' },
            halfAMinute: 'يىرىم مىنۇت',
            lessThanXMinutes: { one: 'بىر مىنۇت ئىچىدە', other: 'مىنۇت ئىچىدە {{count}}' },
            xMinutes: { one: 'بىر مىنۇت', other: 'مىنۇت {{count}}' },
            aboutXHours: { one: 'تەخمىنەن بىر سائەت', other: 'سائەت {{count}} تەخمىنەن' },
            xHours: { one: 'بىر سائەت', other: 'سائەت {{count}}' },
            xDays: { one: 'بىر كۈن', other: 'كۈن {{count}}' },
            aboutXWeeks: { one: 'تەخمىنەن بىرھەپتە', other: 'ھەپتە {{count}} تەخمىنەن' },
            xWeeks: { one: 'بىرھەپتە', other: 'ھەپتە {{count}}' },
            aboutXMonths: { one: 'تەخمىنەن بىر ئاي', other: 'ئاي {{count}} تەخمىنەن' },
            xMonths: { one: 'بىر ئاي', other: 'ئاي {{count}}' },
            aboutXYears: { one: 'تەخمىنەن بىر يىل', other: 'يىل {{count}} تەخمىنەن' },
            xYears: { one: 'بىر يىل', other: 'يىل {{count}}' },
            overXYears: { one: 'بىر يىلدىن ئارتۇق', other: 'يىلدىن ئارتۇق {{count}}' },
            almostXYears: { one: 'ئاساسەن بىر يىل', other: 'يىل {{count}} ئاساسەن' },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? result
                  : result + ' بولدى'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
