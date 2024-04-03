'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [11941],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/km/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: 'តិចជាង {{count}} វិនាទី',
            xSeconds: '{{count}} វិនាទី',
            halfAMinute: 'កន្លះនាទី',
            lessThanXMinutes: 'តិចជាង {{count}} នាទី',
            xMinutes: '{{count}} នាទី',
            aboutXHours: 'ប្រហែល {{count}} ម៉ោង',
            xHours: '{{count}} ម៉ោង',
            xDays: '{{count}} ថ្ងៃ',
            aboutXWeeks: 'ប្រហែល {{count}} សប្តាហ៍',
            xWeeks: '{{count}} សប្តាហ៍',
            aboutXMonths: 'ប្រហែល {{count}} ខែ',
            xMonths: '{{count}} ខែ',
            aboutXYears: 'ប្រហែល {{count}} ឆ្នាំ',
            xYears: '{{count}} ឆ្នាំ',
            overXYears: 'ជាង {{count}} ឆ្នាំ',
            almostXYears: 'ជិត {{count}} ឆ្នាំ',
          },
          _default = function formatDistance(token, count, options) {
            var result = formatDistanceLocale[token];
            return (
              'number' == typeof count && (result = result.replace('{{count}}', count.toString())),
              null != options && options.addSuffix
                ? options.comparison && options.comparison > 0
                  ? 'ក្នុងរយៈពេល ' + result
                  : result + 'មុន'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
