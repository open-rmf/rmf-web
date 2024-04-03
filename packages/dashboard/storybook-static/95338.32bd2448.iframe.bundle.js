'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [95338],
  {
    '../../node_modules/.pnpm/date-fns@2.30.0/node_modules/date-fns/locale/hu/_lib/formatDistance/index.js':
      (module, exports) => {
        Object.defineProperty(exports, '__esModule', { value: !0 }), (exports.default = void 0);
        var translations = {
            about: 'körülbelül',
            over: 'több mint',
            almost: 'majdnem',
            lessthan: 'kevesebb mint',
          },
          withoutSuffixes = {
            xseconds: ' másodperc',
            halfaminute: 'fél perc',
            xminutes: ' perc',
            xhours: ' óra',
            xdays: ' nap',
            xweeks: ' hét',
            xmonths: ' hónap',
            xyears: ' év',
          },
          withSuffixes = {
            xseconds: { '-1': ' másodperccel ezelőtt', 1: ' másodperc múlva', 0: ' másodperce' },
            halfaminute: { '-1': 'fél perccel ezelőtt', 1: 'fél perc múlva', 0: 'fél perce' },
            xminutes: { '-1': ' perccel ezelőtt', 1: ' perc múlva', 0: ' perce' },
            xhours: { '-1': ' órával ezelőtt', 1: ' óra múlva', 0: ' órája' },
            xdays: { '-1': ' nappal ezelőtt', 1: ' nap múlva', 0: ' napja' },
            xweeks: { '-1': ' héttel ezelőtt', 1: ' hét múlva', 0: ' hete' },
            xmonths: { '-1': ' hónappal ezelőtt', 1: ' hónap múlva', 0: ' hónapja' },
            xyears: { '-1': ' évvel ezelőtt', 1: ' év múlva', 0: ' éve' },
          },
          _default = function formatDistance(token, count, options) {
            var adverb = token.match(/about|over|almost|lessthan/i),
              unit = adverb ? token.replace(adverb[0], '') : token,
              addSuffix = !0 === (null == options ? void 0 : options.addSuffix),
              key = unit.toLowerCase(),
              comparison = (null == options ? void 0 : options.comparison) || 0,
              translated = addSuffix ? withSuffixes[key][comparison] : withoutSuffixes[key],
              result = 'halfaminute' === key ? translated : count + translated;
            if (adverb) {
              var adv = adverb[0].toLowerCase();
              result = translations[adv] + ' ' + result;
            }
            return result;
          };
        (exports.default = _default), (module.exports = exports.default);
      },
  },
]);
