'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [19156],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/el/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              one: 'λιγότερο από ένα δευτερόλεπτο',
              other: 'λιγότερο από {{count}} δευτερόλεπτα',
            },
            xSeconds: { one: '1 δευτερόλεπτο', other: '{{count}} δευτερόλεπτα' },
            halfAMinute: 'μισό λεπτό',
            lessThanXMinutes: {
              one: 'λιγότερο από ένα λεπτό',
              other: 'λιγότερο από {{count}} λεπτά',
            },
            xMinutes: { one: '1 λεπτό', other: '{{count}} λεπτά' },
            aboutXHours: { one: 'περίπου 1 ώρα', other: 'περίπου {{count}} ώρες' },
            xHours: { one: '1 ώρα', other: '{{count}} ώρες' },
            xDays: { one: '1 ημέρα', other: '{{count}} ημέρες' },
            aboutXWeeks: { one: 'περίπου 1 εβδομάδα', other: 'περίπου {{count}} εβδομάδες' },
            xWeeks: { one: '1 εβδομάδα', other: '{{count}} εβδομάδες' },
            aboutXMonths: { one: 'περίπου 1 μήνας', other: 'περίπου {{count}} μήνες' },
            xMonths: { one: '1 μήνας', other: '{{count}} μήνες' },
            aboutXYears: { one: 'περίπου 1 χρόνο', other: 'περίπου {{count}} χρόνια' },
            xYears: { one: '1 χρόνο', other: '{{count}} χρόνια' },
            overXYears: { one: 'πάνω από 1 χρόνο', other: 'πάνω από {{count}} χρόνια' },
            almostXYears: { one: 'περίπου 1 χρόνο', other: 'περίπου {{count}} χρόνια' },
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
                  ? 'σε ' + result
                  : result + ' πριν'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
