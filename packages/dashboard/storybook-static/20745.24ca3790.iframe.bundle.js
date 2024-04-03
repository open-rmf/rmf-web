'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [20745],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/uk/_lib/formatDistance/index.js':
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
                  : 'за ' + declension(scheme.regular, count)
                : scheme.past
                  ? declension(scheme.past, count)
                  : declension(scheme.regular, count) + ' тому'
              : declension(scheme.regular, count);
          };
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: buildLocalizeTokenFn({
              regular: {
                one: 'менше секунди',
                singularNominative: 'менше {{count}} секунди',
                singularGenitive: 'менше {{count}} секунд',
                pluralGenitive: 'менше {{count}} секунд',
              },
              future: {
                one: 'менше, ніж за секунду',
                singularNominative: 'менше, ніж за {{count}} секунду',
                singularGenitive: 'менше, ніж за {{count}} секунди',
                pluralGenitive: 'менше, ніж за {{count}} секунд',
              },
            }),
            xSeconds: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} секунда',
                singularGenitive: '{{count}} секунди',
                pluralGenitive: '{{count}} секунд',
              },
              past: {
                singularNominative: '{{count}} секунду тому',
                singularGenitive: '{{count}} секунди тому',
                pluralGenitive: '{{count}} секунд тому',
              },
              future: {
                singularNominative: 'за {{count}} секунду',
                singularGenitive: 'за {{count}} секунди',
                pluralGenitive: 'за {{count}} секунд',
              },
            }),
            halfAMinute: function halfAtMinute(_, options) {
              return options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'за півхвилини'
                  : 'півхвилини тому'
                : 'півхвилини';
            },
            lessThanXMinutes: buildLocalizeTokenFn({
              regular: {
                one: 'менше хвилини',
                singularNominative: 'менше {{count}} хвилини',
                singularGenitive: 'менше {{count}} хвилин',
                pluralGenitive: 'менше {{count}} хвилин',
              },
              future: {
                one: 'менше, ніж за хвилину',
                singularNominative: 'менше, ніж за {{count}} хвилину',
                singularGenitive: 'менше, ніж за {{count}} хвилини',
                pluralGenitive: 'менше, ніж за {{count}} хвилин',
              },
            }),
            xMinutes: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} хвилина',
                singularGenitive: '{{count}} хвилини',
                pluralGenitive: '{{count}} хвилин',
              },
              past: {
                singularNominative: '{{count}} хвилину тому',
                singularGenitive: '{{count}} хвилини тому',
                pluralGenitive: '{{count}} хвилин тому',
              },
              future: {
                singularNominative: 'за {{count}} хвилину',
                singularGenitive: 'за {{count}} хвилини',
                pluralGenitive: 'за {{count}} хвилин',
              },
            }),
            aboutXHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'близько {{count}} години',
                singularGenitive: 'близько {{count}} годин',
                pluralGenitive: 'близько {{count}} годин',
              },
              future: {
                singularNominative: 'приблизно за {{count}} годину',
                singularGenitive: 'приблизно за {{count}} години',
                pluralGenitive: 'приблизно за {{count}} годин',
              },
            }),
            xHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} годину',
                singularGenitive: '{{count}} години',
                pluralGenitive: '{{count}} годин',
              },
            }),
            xDays: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} день',
                singularGenitive: '{{count}} днi',
                pluralGenitive: '{{count}} днів',
              },
            }),
            aboutXWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'близько {{count}} тижня',
                singularGenitive: 'близько {{count}} тижнів',
                pluralGenitive: 'близько {{count}} тижнів',
              },
              future: {
                singularNominative: 'приблизно за {{count}} тиждень',
                singularGenitive: 'приблизно за {{count}} тижні',
                pluralGenitive: 'приблизно за {{count}} тижнів',
              },
            }),
            xWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} тиждень',
                singularGenitive: '{{count}} тижні',
                pluralGenitive: '{{count}} тижнів',
              },
            }),
            aboutXMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'близько {{count}} місяця',
                singularGenitive: 'близько {{count}} місяців',
                pluralGenitive: 'близько {{count}} місяців',
              },
              future: {
                singularNominative: 'приблизно за {{count}} місяць',
                singularGenitive: 'приблизно за {{count}} місяці',
                pluralGenitive: 'приблизно за {{count}} місяців',
              },
            }),
            xMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} місяць',
                singularGenitive: '{{count}} місяці',
                pluralGenitive: '{{count}} місяців',
              },
            }),
            aboutXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'близько {{count}} року',
                singularGenitive: 'близько {{count}} років',
                pluralGenitive: 'близько {{count}} років',
              },
              future: {
                singularNominative: 'приблизно за {{count}} рік',
                singularGenitive: 'приблизно за {{count}} роки',
                pluralGenitive: 'приблизно за {{count}} років',
              },
            }),
            xYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} рік',
                singularGenitive: '{{count}} роки',
                pluralGenitive: '{{count}} років',
              },
            }),
            overXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'більше {{count}} року',
                singularGenitive: 'більше {{count}} років',
                pluralGenitive: 'більше {{count}} років',
              },
              future: {
                singularNominative: 'більше, ніж за {{count}} рік',
                singularGenitive: 'більше, ніж за {{count}} роки',
                pluralGenitive: 'більше, ніж за {{count}} років',
              },
            }),
            almostXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'майже {{count}} рік',
                singularGenitive: 'майже {{count}} роки',
                pluralGenitive: 'майже {{count}} років',
              },
              future: {
                singularNominative: 'майже за {{count}} рік',
                singularGenitive: 'майже за {{count}} роки',
                pluralGenitive: 'майже за {{count}} років',
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
