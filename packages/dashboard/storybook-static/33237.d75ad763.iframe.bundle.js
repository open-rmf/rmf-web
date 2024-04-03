'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [33237],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/is/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'minna en 1 sekúnda', other: 'minna en {{count}} sekúndur' },
            xSeconds: { one: '1 sekúnda', other: '{{count}} sekúndur' },
            halfAMinute: 'hálf mínúta',
            lessThanXMinutes: { one: 'minna en 1 mínúta', other: 'minna en {{count}} mínútur' },
            xMinutes: { one: '1 mínúta', other: '{{count}} mínútur' },
            aboutXHours: { one: 'u.þ.b. 1 klukkustund', other: 'u.þ.b. {{count}} klukkustundir' },
            xHours: { one: '1 klukkustund', other: '{{count}} klukkustundir' },
            xDays: { one: '1 dagur', other: '{{count}} dagar' },
            aboutXWeeks: { one: 'um viku', other: 'um {{count}} vikur' },
            xWeeks: { one: '1 viku', other: '{{count}} vikur' },
            aboutXMonths: { one: 'u.þ.b. 1 mánuður', other: 'u.þ.b. {{count}} mánuðir' },
            xMonths: { one: '1 mánuður', other: '{{count}} mánuðir' },
            aboutXYears: { one: 'u.þ.b. 1 ár', other: 'u.þ.b. {{count}} ár' },
            xYears: { one: '1 ár', other: '{{count}} ár' },
            overXYears: { one: 'meira en 1 ár', other: 'meira en {{count}} ár' },
            almostXYears: { one: 'næstum 1 ár', other: 'næstum {{count}} ár' },
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
                    : tokenValue.other.replace('{{count}}', count.toString())),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'í ' + result
                  : result + ' síðan'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
