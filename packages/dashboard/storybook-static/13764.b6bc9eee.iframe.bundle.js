'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [13764],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/ru/_lib/formatDistance/index.js':
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
            return null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? scheme.future
                  ? declension(scheme.future, count)
                  : 'через ' + declension(scheme.regular, count)
                : scheme.past
                  ? declension(scheme.past, count)
                  : declension(scheme.regular, count) + ' назад'
              : declension(scheme.regular, count);
          };
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: buildLocalizeTokenFn({
              regular: {
                one: 'меньше секунды',
                singularNominative: 'меньше {{count}} секунды',
                singularGenitive: 'меньше {{count}} секунд',
                pluralGenitive: 'меньше {{count}} секунд',
              },
              future: {
                one: 'меньше, чем через секунду',
                singularNominative: 'меньше, чем через {{count}} секунду',
                singularGenitive: 'меньше, чем через {{count}} секунды',
                pluralGenitive: 'меньше, чем через {{count}} секунд',
              },
            }),
            xSeconds: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} секунда',
                singularGenitive: '{{count}} секунды',
                pluralGenitive: '{{count}} секунд',
              },
              past: {
                singularNominative: '{{count}} секунду назад',
                singularGenitive: '{{count}} секунды назад',
                pluralGenitive: '{{count}} секунд назад',
              },
              future: {
                singularNominative: 'через {{count}} секунду',
                singularGenitive: 'через {{count}} секунды',
                pluralGenitive: 'через {{count}} секунд',
              },
            }),
            halfAMinute: function halfAMinute(_count, options) {
              return null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'через полминуты'
                  : 'полминуты назад'
                : 'полминуты';
            },
            lessThanXMinutes: buildLocalizeTokenFn({
              regular: {
                one: 'меньше минуты',
                singularNominative: 'меньше {{count}} минуты',
                singularGenitive: 'меньше {{count}} минут',
                pluralGenitive: 'меньше {{count}} минут',
              },
              future: {
                one: 'меньше, чем через минуту',
                singularNominative: 'меньше, чем через {{count}} минуту',
                singularGenitive: 'меньше, чем через {{count}} минуты',
                pluralGenitive: 'меньше, чем через {{count}} минут',
              },
            }),
            xMinutes: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} минута',
                singularGenitive: '{{count}} минуты',
                pluralGenitive: '{{count}} минут',
              },
              past: {
                singularNominative: '{{count}} минуту назад',
                singularGenitive: '{{count}} минуты назад',
                pluralGenitive: '{{count}} минут назад',
              },
              future: {
                singularNominative: 'через {{count}} минуту',
                singularGenitive: 'через {{count}} минуты',
                pluralGenitive: 'через {{count}} минут',
              },
            }),
            aboutXHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} часа',
                singularGenitive: 'около {{count}} часов',
                pluralGenitive: 'около {{count}} часов',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} час',
                singularGenitive: 'приблизительно через {{count}} часа',
                pluralGenitive: 'приблизительно через {{count}} часов',
              },
            }),
            xHours: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} час',
                singularGenitive: '{{count}} часа',
                pluralGenitive: '{{count}} часов',
              },
            }),
            xDays: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} день',
                singularGenitive: '{{count}} дня',
                pluralGenitive: '{{count}} дней',
              },
            }),
            aboutXWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} недели',
                singularGenitive: 'около {{count}} недель',
                pluralGenitive: 'около {{count}} недель',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} неделю',
                singularGenitive: 'приблизительно через {{count}} недели',
                pluralGenitive: 'приблизительно через {{count}} недель',
              },
            }),
            xWeeks: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} неделя',
                singularGenitive: '{{count}} недели',
                pluralGenitive: '{{count}} недель',
              },
            }),
            aboutXMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} месяца',
                singularGenitive: 'около {{count}} месяцев',
                pluralGenitive: 'около {{count}} месяцев',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} месяц',
                singularGenitive: 'приблизительно через {{count}} месяца',
                pluralGenitive: 'приблизительно через {{count}} месяцев',
              },
            }),
            xMonths: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} месяц',
                singularGenitive: '{{count}} месяца',
                pluralGenitive: '{{count}} месяцев',
              },
            }),
            aboutXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'около {{count}} года',
                singularGenitive: 'около {{count}} лет',
                pluralGenitive: 'около {{count}} лет',
              },
              future: {
                singularNominative: 'приблизительно через {{count}} год',
                singularGenitive: 'приблизительно через {{count}} года',
                pluralGenitive: 'приблизительно через {{count}} лет',
              },
            }),
            xYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: '{{count}} год',
                singularGenitive: '{{count}} года',
                pluralGenitive: '{{count}} лет',
              },
            }),
            overXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'больше {{count}} года',
                singularGenitive: 'больше {{count}} лет',
                pluralGenitive: 'больше {{count}} лет',
              },
              future: {
                singularNominative: 'больше, чем через {{count}} год',
                singularGenitive: 'больше, чем через {{count}} года',
                pluralGenitive: 'больше, чем через {{count}} лет',
              },
            }),
            almostXYears: buildLocalizeTokenFn({
              regular: {
                singularNominative: 'почти {{count}} год',
                singularGenitive: 'почти {{count}} года',
                pluralGenitive: 'почти {{count}} лет',
              },
              future: {
                singularNominative: 'почти через {{count}} год',
                singularGenitive: 'почти через {{count}} года',
                pluralGenitive: 'почти через {{count}} лет',
              },
            }),
          },
          _default = function formatDistance(token, count, options) {
            return formatDistanceLocale[token](count, options);
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
