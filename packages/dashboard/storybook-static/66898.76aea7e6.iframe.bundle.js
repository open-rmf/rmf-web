'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [66898],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/mn/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: { one: 'секунд хүрэхгүй', other: '{{count}} секунд хүрэхгүй' },
            xSeconds: { one: '1 секунд', other: '{{count}} секунд' },
            halfAMinute: 'хагас минут',
            lessThanXMinutes: { one: 'минут хүрэхгүй', other: '{{count}} минут хүрэхгүй' },
            xMinutes: { one: '1 минут', other: '{{count}} минут' },
            aboutXHours: { one: 'ойролцоогоор 1 цаг', other: 'ойролцоогоор {{count}} цаг' },
            xHours: { one: '1 цаг', other: '{{count}} цаг' },
            xDays: { one: '1 өдөр', other: '{{count}} өдөр' },
            aboutXWeeks: {
              one: 'ойролцоогоор 1 долоо хоног',
              other: 'ойролцоогоор {{count}} долоо хоног',
            },
            xWeeks: { one: '1 долоо хоног', other: '{{count}} долоо хоног' },
            aboutXMonths: { one: 'ойролцоогоор 1 сар', other: 'ойролцоогоор {{count}} сар' },
            xMonths: { one: '1 сар', other: '{{count}} сар' },
            aboutXYears: { one: 'ойролцоогоор 1 жил', other: 'ойролцоогоор {{count}} жил' },
            xYears: { one: '1 жил', other: '{{count}} жил' },
            overXYears: { one: '1 жил гаран', other: '{{count}} жил гаран' },
            almostXYears: { one: 'бараг 1 жил', other: 'бараг {{count}} жил' },
          },
          _default = function formatDistance(token, count, options) {
            var result,
              tokenValue = formatDistanceLocale[token];
            if (
              ((result =
                'string' == typeof tokenValue
                  ? tokenValue
                  : 1 === count
                    ? tokenValue.one
                    : tokenValue.other.replace('{{count}}', String(count))),
              null != options && options.addSuffix)
            ) {
              var words = result.split(' '),
                lastword = words.pop();
              switch (((result = words.join(' ')), lastword)) {
                case 'секунд':
                  result += ' секундийн';
                  break;
                case 'минут':
                  result += ' минутын';
                  break;
                case 'цаг':
                  result += ' цагийн';
                  break;
                case 'өдөр':
                  result += ' өдрийн';
                  break;
                case 'сар':
                  result += ' сарын';
                  break;
                case 'жил':
                  result += ' жилийн';
                  break;
                case 'хоног':
                  result += ' хоногийн';
                  break;
                case 'гаран':
                  result += ' гараны';
                  break;
                case 'хүрэхгүй':
                  result += ' хүрэхгүй хугацааны';
                  break;
                default:
                  result += lastword + '-н';
              }
              return options.comparison && options.comparison > 0
                ? result + ' дараа'
                : result + ' өмнө';
            }
            return result;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
