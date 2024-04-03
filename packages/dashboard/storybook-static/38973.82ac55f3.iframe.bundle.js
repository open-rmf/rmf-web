'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [38973],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/fr/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'moins d’une seconde', other: 'moins de {{count}} secondes' },
            xSeconds: { one: '1 seconde', other: '{{count}} secondes' },
            halfAMinute: '30 secondes',
            lessThanXMinutes: { one: 'moins d’une minute', other: 'moins de {{count}} minutes' },
            xMinutes: { one: '1 minute', other: '{{count}} minutes' },
            aboutXHours: { one: 'environ 1 heure', other: 'environ {{count}} heures' },
            xHours: { one: '1 heure', other: '{{count}} heures' },
            xDays: { one: '1 jour', other: '{{count}} jours' },
            aboutXWeeks: { one: 'environ 1 semaine', other: 'environ {{count}} semaines' },
            xWeeks: { one: '1 semaine', other: '{{count}} semaines' },
            aboutXMonths: { one: 'environ 1 mois', other: 'environ {{count}} mois' },
            xMonths: { one: '1 mois', other: '{{count}} mois' },
            aboutXYears: { one: 'environ 1 an', other: 'environ {{count}} ans' },
            xYears: { one: '1 an', other: '{{count}} ans' },
            overXYears: { one: 'plus d’un an', other: 'plus de {{count}} ans' },
            almostXYears: { one: 'presqu’un an', other: 'presque {{count}} ans' },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              form = formatDistanceLocale[token];
            return (
              (result =
                'string' == typeof form
                  ? form
                  : 1 === count
                    ? form.one
                    : form.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'dans ' + result
                  : 'il y a ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
