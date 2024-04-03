'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [13355],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sk/_lib/formatDistance/index.js':
      (module, exports) => {
        function declension(scheme, count, time) {
          var group = (function declensionGroup(scheme, count) {
            return 1 === count && scheme.one
              ? scheme.one
              : count >= 2 && count <= 4 && scheme.twoFour
                ? scheme.twoFour
                : scheme.other;
          })(scheme, count);
          return group[time].replace('{{count}}', String(count));
        }
        function prefixPreposition(preposition) {
          var translation = '';
          return (
            'almost' === preposition && (translation = 'takmer'),
            'about' === preposition && (translation = 'približne'),
            translation.length > 0 ? translation + ' ' : ''
          );
        }
        function suffixPreposition(preposition) {
          var translation = '';
          return (
            'lessThan' === preposition && (translation = 'menej než'),
            'over' === preposition && (translation = 'viac než'),
            translation.length > 0 ? translation + ' ' : ''
          );
        }
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            xSeconds: {
              one: { present: 'sekunda', past: 'sekundou', future: 'sekundu' },
              twoFour: {
                present: '{{count}} sekundy',
                past: '{{count}} sekundami',
                future: '{{count}} sekundy',
              },
              other: {
                present: '{{count}} sekúnd',
                past: '{{count}} sekundami',
                future: '{{count}} sekúnd',
              },
            },
            halfAMinute: {
              other: { present: 'pol minúty', past: 'pol minútou', future: 'pol minúty' },
            },
            xMinutes: {
              one: { present: 'minúta', past: 'minútou', future: 'minútu' },
              twoFour: {
                present: '{{count}} minúty',
                past: '{{count}} minútami',
                future: '{{count}} minúty',
              },
              other: {
                present: '{{count}} minút',
                past: '{{count}} minútami',
                future: '{{count}} minút',
              },
            },
            xHours: {
              one: { present: 'hodina', past: 'hodinou', future: 'hodinu' },
              twoFour: {
                present: '{{count}} hodiny',
                past: '{{count}} hodinami',
                future: '{{count}} hodiny',
              },
              other: {
                present: '{{count}} hodín',
                past: '{{count}} hodinami',
                future: '{{count}} hodín',
              },
            },
            xDays: {
              one: { present: 'deň', past: 'dňom', future: 'deň' },
              twoFour: {
                present: '{{count}} dni',
                past: '{{count}} dňami',
                future: '{{count}} dni',
              },
              other: { present: '{{count}} dní', past: '{{count}} dňami', future: '{{count}} dní' },
            },
            xWeeks: {
              one: { present: 'týždeň', past: 'týždňom', future: 'týždeň' },
              twoFour: {
                present: '{{count}} týždne',
                past: '{{count}} týždňami',
                future: '{{count}} týždne',
              },
              other: {
                present: '{{count}} týždňov',
                past: '{{count}} týždňami',
                future: '{{count}} týždňov',
              },
            },
            xMonths: {
              one: { present: 'mesiac', past: 'mesiacom', future: 'mesiac' },
              twoFour: {
                present: '{{count}} mesiace',
                past: '{{count}} mesiacmi',
                future: '{{count}} mesiace',
              },
              other: {
                present: '{{count}} mesiacov',
                past: '{{count}} mesiacmi',
                future: '{{count}} mesiacov',
              },
            },
            xYears: {
              one: { present: 'rok', past: 'rokom', future: 'rok' },
              twoFour: {
                present: '{{count}} roky',
                past: '{{count}} rokmi',
                future: '{{count}} roky',
              },
              other: {
                present: '{{count}} rokov',
                past: '{{count}} rokmi',
                future: '{{count}} rokov',
              },
            },
          },
          _default = function formatDistance(token, count, options) {
            var preposition =
                (function extractPreposition(token) {
                  return ['lessThan', 'about', 'over', 'almost'].filter(function (preposition) {
                    return !!token.match(new RegExp('^' + preposition));
                  })[0];
                })(token) || '',
              key = (function lowercaseFirstLetter(string) {
                return string.charAt(0).toLowerCase() + string.slice(1);
              })(token.substring(preposition.length)),
              scheme = formatDistanceLocale[key];
            return null != options && options.addSuffix
              ? options.comparison && options.comparison > 0
                ? prefixPreposition(preposition) +
                  'o ' +
                  suffixPreposition(preposition) +
                  declension(scheme, count, 'future')
                : prefixPreposition(preposition) +
                  'pred ' +
                  suffixPreposition(preposition) +
                  declension(scheme, count, 'past')
              : prefixPreposition(preposition) +
                  suffixPreposition(preposition) +
                  declension(scheme, count, 'present');
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
