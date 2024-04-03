'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [43246],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/de/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              standalone: { one: 'weniger als 1 Sekunde', other: 'weniger als {{count}} Sekunden' },
              withPreposition: {
                one: 'weniger als 1 Sekunde',
                other: 'weniger als {{count}} Sekunden',
              },
            },
            xSeconds: {
              standalone: { one: '1 Sekunde', other: '{{count}} Sekunden' },
              withPreposition: { one: '1 Sekunde', other: '{{count}} Sekunden' },
            },
            halfAMinute: { standalone: 'halbe Minute', withPreposition: 'halben Minute' },
            lessThanXMinutes: {
              standalone: { one: 'weniger als 1 Minute', other: 'weniger als {{count}} Minuten' },
              withPreposition: {
                one: 'weniger als 1 Minute',
                other: 'weniger als {{count}} Minuten',
              },
            },
            xMinutes: {
              standalone: { one: '1 Minute', other: '{{count}} Minuten' },
              withPreposition: { one: '1 Minute', other: '{{count}} Minuten' },
            },
            aboutXHours: {
              standalone: { one: 'etwa 1 Stunde', other: 'etwa {{count}} Stunden' },
              withPreposition: { one: 'etwa 1 Stunde', other: 'etwa {{count}} Stunden' },
            },
            xHours: {
              standalone: { one: '1 Stunde', other: '{{count}} Stunden' },
              withPreposition: { one: '1 Stunde', other: '{{count}} Stunden' },
            },
            xDays: {
              standalone: { one: '1 Tag', other: '{{count}} Tage' },
              withPreposition: { one: '1 Tag', other: '{{count}} Tagen' },
            },
            aboutXWeeks: {
              standalone: { one: 'etwa 1 Woche', other: 'etwa {{count}} Wochen' },
              withPreposition: { one: 'etwa 1 Woche', other: 'etwa {{count}} Wochen' },
            },
            xWeeks: {
              standalone: { one: '1 Woche', other: '{{count}} Wochen' },
              withPreposition: { one: '1 Woche', other: '{{count}} Wochen' },
            },
            aboutXMonths: {
              standalone: { one: 'etwa 1 Monat', other: 'etwa {{count}} Monate' },
              withPreposition: { one: 'etwa 1 Monat', other: 'etwa {{count}} Monaten' },
            },
            xMonths: {
              standalone: { one: '1 Monat', other: '{{count}} Monate' },
              withPreposition: { one: '1 Monat', other: '{{count}} Monaten' },
            },
            aboutXYears: {
              standalone: { one: 'etwa 1 Jahr', other: 'etwa {{count}} Jahre' },
              withPreposition: { one: 'etwa 1 Jahr', other: 'etwa {{count}} Jahren' },
            },
            xYears: {
              standalone: { one: '1 Jahr', other: '{{count}} Jahre' },
              withPreposition: { one: '1 Jahr', other: '{{count}} Jahren' },
            },
            overXYears: {
              standalone: { one: 'mehr als 1 Jahr', other: 'mehr als {{count}} Jahre' },
              withPreposition: { one: 'mehr als 1 Jahr', other: 'mehr als {{count}} Jahren' },
            },
            almostXYears: {
              standalone: { one: 'fast 1 Jahr', other: 'fast {{count}} Jahre' },
              withPreposition: { one: 'fast 1 Jahr', other: 'fast {{count}} Jahren' },
            },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue =
                null != options && options.addSuffix
                  ? formatDistanceLocale[token].withPreposition
                  : formatDistanceLocale[token].standalone;
            return (
              (result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'in ' + result
                  : 'vor ' + result
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
