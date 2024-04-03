'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [62127],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/oc/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'mens d’una segonda', other: 'mens de {{count}} segondas' },
            xSeconds: { one: '1 segonda', other: '{{count}} segondas' },
            halfAMinute: '30 segondas',
            lessThanXMinutes: { one: 'mens d’una minuta', other: 'mens de {{count}} minutas' },
            xMinutes: { one: '1 minuta', other: '{{count}} minutas' },
            aboutXHours: { one: 'environ 1 ora', other: 'environ {{count}} oras' },
            xHours: { one: '1 ora', other: '{{count}} oras' },
            xDays: { one: '1 jorn', other: '{{count}} jorns' },
            aboutXWeeks: { one: 'environ 1 setmana', other: 'environ {{count}} setmanas' },
            xWeeks: { one: '1 setmana', other: '{{count}} setmanas' },
            aboutXMonths: { one: 'environ 1 mes', other: 'environ {{count}} meses' },
            xMonths: { one: '1 mes', other: '{{count}} meses' },
            aboutXYears: { one: 'environ 1 an', other: 'environ {{count}} ans' },
            xYears: { one: '1 an', other: '{{count}} ans' },
            overXYears: { one: 'mai d’un an', other: 'mai de {{count}} ans' },
            almostXYears: { one: 'gaireben un an', other: 'gaireben {{count}} ans' },
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
                  ? 'd’aquí ' + result
                  : 'fa ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
