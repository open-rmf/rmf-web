import React from 'react';

export const TooltipContext = React.createContext({
  showTooltips: true,
  toggleTooltips: () => {},
});
