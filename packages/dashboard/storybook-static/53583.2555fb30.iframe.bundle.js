'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [53583],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/formatDistance/index.js':
      (module, exports) => {
        function declension(scheme, count) {
          if (void 0 !== scheme.one && 1 === count) return scheme.one;
          var rem10 = count % 10,
            rem100 = count % 100;
          return 1 === rem10 && 11 !== rem100
            ? scheme.singularNominative.replace('{{count}}', String(count))
            : rem10 >= 2 && rem10 <= 4 && (rem100 < 10 || rem100 > 20)
              ? scheme.singularGenitive.replace('{{count}}', String(count))
              : scheme.pluralGenitive.replace('{{count}}', String(count));
        }
        function buildLocalizeTokenFn(scheme) {
          return function (count, options) {
            return options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? scheme.future
                  ? declension(scheme.future, count)
                  : 'праз ' + declension(scheme.regular, count)
                : scheme.past
                  ? declension(scheme.past, count)
                  : declension(scheme.regular, count) + ' таму'
              : declension(scheme.regular, count);
          };
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: buildLocalizeTokenFn({
              regular: {
                one: 'менш за секунду',
                singularNominative: 'менш за {{count}} секунду',
                singularGenitive: 'менш за {{count}} секунды',
                pluralGenitive: 'менш за {{count}} секунд',
              },
              future: {
                one: 'менш, чым праз секунду',
                singularNominative: 'менш, чым праз {{count}} секунду',
                singularGenitive: 'менш, чым праз {{count}} секунды',
                pluralGenitive: 'менш, чым праз {{count}} секунд',
              },
            }),
            xSeconds: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} секунда',
                singularGenitive: '{{count}} секунды',
                pluralGenitive: '{{count}} секунд',
              },
              past: {
                singularNominative: '{{count}} секунду таму',
                singularGenitive: '{{count}} секунды таму',
                pluralGenitive: '{{count}} секунд таму',
              },
              future: {
                singularNominative: 'праз {{count}} секунду',
                singularGenitive: 'праз {{count}} секунды',
                pluralGenitive: 'праз {{count}} секунд',
              },
            }),
            halfAMinute: function halfAMinute(_, options) {
              return options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'праз паўхвіліны'
                  : 'паўхвіліны таму'
                : 'паўхвіліны';
            },
            lessThanXMinutes: buildLocalizeTokenFn({
              regular: {
                one: 'менш за хвіліну',
                singularNominative: 'менш за {{count}} хвіліну',
                singularGenitive: 'менш за {{count}} хвіліны',
                pluralGenitive: 'менш за {{count}} хвілін',
              },
              future: {
                one: 'менш, чым праз хвіліну',
                singularNominative: 'менш, чым праз {{count}} хвіліну',
                singularGenitive: 'менш, чым праз {{count}} хвіліны',
                pluralGenitive: 'менш, чым праз {{count}} хвілін',
              },
            }),
            xMinutes: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} хвіліна',
                singularGenitive: '{{count}} хвіліны',
                pluralGenitive: '{{count}} хвілін',
              },
              past: {
                singularNominative: '{{count}} хвіліну таму',
                singularGenitive: '{{count}} хвіліны таму',
                pluralGenitive: '{{count}} хвілін таму',
              },
              future: {
                singularNominative: 'праз {{count}} хвіліну',
                singularGenitive: 'праз {{count}} хвіліны',
                pluralGenitive: 'праз {{count}} хвілін',
              },
            }),
            aboutXHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} гадзіны',
                singularGenitive: 'каля {{count}} гадзін',
                pluralGenitive: 'каля {{count}} гадзін',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} гадзіну',
                singularGenitive: 'прыблізна праз {{count}} гадзіны',
                pluralGenitive: 'прыблізна праз {{count}} гадзін',
              },
            }),
            xHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} гадзіна',
                singularGenitive: '{{count}} гадзіны',
                pluralGenitive: '{{count}} гадзін',
              },
              past: {
                singularNominative: '{{count}} гадзіну таму',
                singularGenitive: '{{count}} гадзіны таму',
                pluralGenitive: '{{count}} гадзін таму',
              },
              future: {
                singularNominative: 'праз {{count}} гадзіну',
                singularGenitive: 'праз {{count}} гадзіны',
                pluralGenitive: 'праз {{count}} гадзін',
              },
            }),
            xDays: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} дзень',
                singularGenitive: '{{count}} дні',
                pluralGenitive: '{{count}} дзён',
              },
            }),
            aboutXWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} месяца',
                singularGenitive: 'каля {{count}} месяцаў',
                pluralGenitive: 'каля {{count}} месяцаў',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} месяц',
                singularGenitive: 'прыблізна праз {{count}} месяцы',
                pluralGenitive: 'прыблізна праз {{count}} месяцаў',
              },
            }),
            xWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} месяц',
                singularGenitive: '{{count}} месяцы',
                pluralGenitive: '{{count}} месяцаў',
              },
            }),
            aboutXMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} месяца',
                singularGenitive: 'каля {{count}} месяцаў',
                pluralGenitive: 'каля {{count}} месяцаў',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} месяц',
                singularGenitive: 'прыблізна праз {{count}} месяцы',
                pluralGenitive: 'прыблізна праз {{count}} месяцаў',
              },
            }),
            xMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} месяц',
                singularGenitive: '{{count}} месяцы',
                pluralGenitive: '{{count}} месяцаў',
              },
            }),
            aboutXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'каля {{count}} года',
                singularGenitive: 'каля {{count}} гадоў',
                pluralGenitive: 'каля {{count}} гадоў',
              },
              future: {
                singularNominative: 'прыблізна праз {{count}} год',
                singularGenitive: 'прыблізна праз {{count}} гады',
                pluralGenitive: 'прыблізна праз {{count}} гадоў',
              },
            }),
            xYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} год',
                singularGenitive: '{{count}} гады',
                pluralGenitive: '{{count}} гадоў',
              },
            }),
            overXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'больш за {{count}} год',
                singularGenitive: 'больш за {{count}} гады',
                pluralGenitive: 'больш за {{count}} гадоў',
              },
              future: {
                singularNominative: 'больш, чым праз {{count}} год',
                singularGenitive: 'больш, чым праз {{count}} гады',
                pluralGenitive: 'больш, чым праз {{count}} гадоў',
              },
            }),
            almostXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'амаль {{count}} год',
                singularGenitive: 'амаль {{count}} гады',
                pluralGenitive: 'амаль {{count}} гадоў',
              },
              future: {
                singularNominative: 'амаль праз {{count}} год',
                singularGenitive: 'амаль праз {{count}} гады',
                pluralGenitive: 'амаль праз {{count}} гадоў',
              },
            }),
          },
          _default = function formatDistance(token, count, options) {
            return (options = options || {}), formatDistanceLocale[token](count, options);
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
