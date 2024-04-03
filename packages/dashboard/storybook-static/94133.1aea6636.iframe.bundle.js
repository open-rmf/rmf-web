'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [94133],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/pl/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
          lessThanXSeconds: {
            one: {
              regular: 'mniej niż sekunda',
              past: 'mniej niż sekundę',
              future: 'mniej niż sekundę',
            },
            twoFour: 'mniej niż {{count}} sekundy',
            other: 'mniej niż {{count}} sekund',
          },
          xSeconds: {
            one: { regular: 'sekunda', past: 'sekundę', future: 'sekundę' },
            twoFour: '{{count}} sekundy',
            other: '{{count}} sekund',
          },
          halfAMinute: { one: 'pół minuty', twoFour: 'pół minuty', other: 'pół minuty' },
          lessThanXMinutes: {
            one: {
              regular: 'mniej niż minuta',
              past: 'mniej niż minutę',
              future: 'mniej niż minutę',
            },
            twoFour: 'mniej niż {{count}} minuty',
            other: 'mniej niż {{count}} minut',
          },
          xMinutes: {
            one: { regular: 'minuta', past: 'minutę', future: 'minutę' },
            twoFour: '{{count}} minuty',
            other: '{{count}} minut',
          },
          aboutXHours: {
            one: { regular: 'około godziny', past: 'około godziny', future: 'około godzinę' },
            twoFour: 'około {{count}} godziny',
            other: 'około {{count}} godzin',
          },
          xHours: {
            one: { regular: 'godzina', past: 'godzinę', future: 'godzinę' },
            twoFour: '{{count}} godziny',
            other: '{{count}} godzin',
          },
          xDays: {
            one: { regular: 'dzień', past: 'dzień', future: '1 dzień' },
            twoFour: '{{count}} dni',
            other: '{{count}} dni',
          },
          aboutXWeeks: {
            one: 'około tygodnia',
            twoFour: 'około {{count}} tygodni',
            other: 'około {{count}} tygodni',
          },
          xWeeks: { one: 'tydzień', twoFour: '{{count}} tygodnie', other: '{{count}} tygodni' },
          aboutXMonths: {
            one: 'około miesiąc',
            twoFour: 'około {{count}} miesiące',
            other: 'około {{count}} miesięcy',
          },
          xMonths: { one: 'miesiąc', twoFour: '{{count}} miesiące', other: '{{count}} miesięcy' },
          aboutXYears: {
            one: 'około rok',
            twoFour: 'około {{count}} lata',
            other: 'około {{count}} lat',
          },
          xYears: { one: 'rok', twoFour: '{{count}} lata', other: '{{count}} lat' },
          overXYears: {
            one: 'ponad rok',
            twoFour: 'ponad {{count}} lata',
            other: 'ponad {{count}} lat',
          },
          almostXYears: {
            one: 'prawie rok',
            twoFour: 'prawie {{count}} lata',
            other: 'prawie {{count}} lat',
          },
        };
        function declension(scheme, count, time) {
          var group = (function declensionGroup(scheme, count) {
            if (1 === count) return scheme.one;
            var rem100 = count % 100;
            if (rem100 <= 20 && rem100 > 10) return scheme.other;
            var rem10 = rem100 % 10;
            return rem10 >= 2 && rem10 <= 4 ? scheme.twoFour : scheme.other;
          })(scheme, count);
          return ('string' == typeof group ? group : group[time]).replace(
            '{{count}}',
            String(count),
          );
        }
        var _default = function formatDistance(token, count, options) {
          var scheme = formatDistanceLocale[token];
          return null != options && options.addSuffix
            ? options.comparison && options.comparison > 0
              ? 'za ' + declension(scheme, count, 'future')
              : declension(scheme, count, 'past') + ' temu'
            : declension(scheme, count, 'regular');
        };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
