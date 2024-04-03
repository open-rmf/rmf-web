'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [17088],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/be-tarask/_lib/formatRelative/index.js':
      (module, exports, __webpack_require__) => {
        var _interopRequireDefault = __webpack_require__(
          '../../node_modules/.pnpm/@babel+runtime@7.24.1/node_modules/@babel/runtime/helpers/interopRequireDefault.js',
        ).default;
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var _index = __webpack_require__(
            '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/index.js',
          ),
          _index2 = _interopRequireDefault(
            __webpack_require__(
              '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/_lib/isSameUTCWeek/index.js',
            ),
          ),
          accusativeWeekdays = [
            'нядзелю',
            'панядзелак',
            'аўторак',
            'сераду',
            'чацьвер',
            'пятніцу',
            'суботу',
          ];
        function thisWeek(day) {
          return "'у " + accusativeWeekdays[day] + " а' p";
        }
        var formatRelativeLocale = {
            lastWeek: function lastWeekFormat(dirtyDate, baseDate, options) {
              var date = (0, _index.toDate)(dirtyDate),
                day = date.getUTCDay();
              return (0, _index2.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function lastWeek(day) {
                    var weekday = accusativeWeekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 5:
                      case 6:
                        return "'у мінулую " + weekday + " а' p";
                      case 1:
                      case 2:
                      case 4:
                        return "'у мінулы " + weekday + " а' p";
                    }
                  })(day);
            },
            yesterday: "'учора а' p",
            today: "'сёньня а' p",
            tomorrow: "'заўтра а' p",
            nextWeek: function nextWeekFormat(dirtyDate, baseDate, options) {
              var date = (0, _index.toDate)(dirtyDate),
                day = date.getUTCDay();
              return (0, _index2.default)(date, baseDate, options)
                ? thisWeek(day)
                : (function nextWeek(day) {
                    var weekday = accusativeWeekdays[day];
                    switch (day) {
                      case 0:
                      case 3:
                      case 5:
                      case 6:
                        return "'у наступную " + weekday + " а' p";
                      case 1:
                      case 2:
                      case 4:
                        return "'у наступны " + weekday + " а' p";
                    }
                  })(day);
            },
            other: 'P',
          },
          _default = function formatRelative(token, date, baseDate, options) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date, baseDate, options) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
