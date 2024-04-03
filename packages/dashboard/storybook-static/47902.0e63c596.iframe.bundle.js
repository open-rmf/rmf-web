'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [47902],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/te/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatDistanceLocale = {
            lessThanXSeconds: {
              standalone: { one: 'సెకను కన్నా తక్కువ', other: '{{count}} సెకన్ల కన్నా తక్కువ' },
              withPreposition: { one: 'సెకను', other: '{{count}} సెకన్ల' },
            },
            xSeconds: {
              standalone: { one: 'ఒక సెకను', other: '{{count}} సెకన్ల' },
              withPreposition: { one: 'ఒక సెకను', other: '{{count}} సెకన్ల' },
            },
            halfAMinute: { standalone: 'అర నిమిషం', withPreposition: 'అర నిమిషం' },
            lessThanXMinutes: {
              standalone: {
                one: 'ఒక నిమిషం కన్నా తక్కువ',
                other: '{{count}} నిమిషాల కన్నా తక్కువ',
              },
              withPreposition: { one: 'ఒక నిమిషం', other: '{{count}} నిమిషాల' },
            },
            xMinutes: {
              standalone: { one: 'ఒక నిమిషం', other: '{{count}} నిమిషాలు' },
              withPreposition: { one: 'ఒక నిమిషం', other: '{{count}} నిమిషాల' },
            },
            aboutXHours: {
              standalone: { one: 'సుమారు ఒక గంట', other: 'సుమారు {{count}} గంటలు' },
              withPreposition: { one: 'సుమారు ఒక గంట', other: 'సుమారు {{count}} గంటల' },
            },
            xHours: {
              standalone: { one: 'ఒక గంట', other: '{{count}} గంటలు' },
              withPreposition: { one: 'ఒక గంట', other: '{{count}} గంటల' },
            },
            xDays: {
              standalone: { one: 'ఒక రోజు', other: '{{count}} రోజులు' },
              withPreposition: { one: 'ఒక రోజు', other: '{{count}} రోజుల' },
            },
            aboutXWeeks: {
              standalone: { one: 'సుమారు ఒక వారం', other: 'సుమారు {{count}} వారాలు' },
              withPreposition: { one: 'సుమారు ఒక వారం', other: 'సుమారు {{count}} వారాలల' },
            },
            xWeeks: {
              standalone: { one: 'ఒక వారం', other: '{{count}} వారాలు' },
              withPreposition: { one: 'ఒక వారం', other: '{{count}} వారాలల' },
            },
            aboutXMonths: {
              standalone: { one: 'సుమారు ఒక నెల', other: 'సుమారు {{count}} నెలలు' },
              withPreposition: { one: 'సుమారు ఒక నెల', other: 'సుమారు {{count}} నెలల' },
            },
            xMonths: {
              standalone: { one: 'ఒక నెల', other: '{{count}} నెలలు' },
              withPreposition: { one: 'ఒక నెల', other: '{{count}} నెలల' },
            },
            aboutXYears: {
              standalone: { one: 'సుమారు ఒక సంవత్సరం', other: 'సుమారు {{count}} సంవత్సరాలు' },
              withPreposition: { one: 'సుమారు ఒక సంవత్సరం', other: 'సుమారు {{count}} సంవత్సరాల' },
            },
            xYears: {
              standalone: { one: 'ఒక సంవత్సరం', other: '{{count}} సంవత్సరాలు' },
              withPreposition: { one: 'ఒక సంవత్సరం', other: '{{count}} సంవత్సరాల' },
            },
            overXYears: {
              standalone: { one: 'ఒక సంవత్సరం పైగా', other: '{{count}} సంవత్సరాలకు పైగా' },
              withPreposition: { one: 'ఒక సంవత్సరం', other: '{{count}} సంవత్సరాల' },
            },
            almostXYears: {
              standalone: { one: 'దాదాపు ఒక సంవత్సరం', other: 'దాదాపు {{count}} సంవత్సరాలు' },
              withPreposition: { one: 'దాదాపు ఒక సంవత్సరం', other: 'దాదాపు {{count}} సంవత్సరాల' },
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
                  ? result + 'లో'
                  : result + ' క్రితం'
                : result
            );
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
