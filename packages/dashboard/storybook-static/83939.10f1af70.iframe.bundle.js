'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [83939],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/sr/_lib/formatRelative/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var formatRelativeLocale = {
            lastWeek: function lastWeek(date) {
              switch (date.getUTCDay()) {
                case 0:
                  return "'прошле недеље у' p";
                case 3:
                  return "'прошле среде у' p";
                case 6:
                  return "'прошле суботе у' p";
                default:
                  return "'прошли' EEEE 'у' p";
              }
            },
            yesterday: "'јуче у' p",
            today: "'данас у' p",
            tomorrow: "'сутра у' p",
            nextWeek: function nextWeek(date) {
              switch (date.getUTCDay()) {
                case 0:
                  return "'следеће недеље у' p";
                case 3:
                  return "'следећу среду у' p";
                case 6:
                  return "'следећу суботу у' p";
                default:
                  return "'следећи' EEEE 'у' p";
              }
            },
            other: 'P',
          },
          _default = function formatRelative(token, date, _baseDate, _options) {
            var format = formatRelativeLocale[token];
            return 'function' == typeof format ? format(date) : format;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
