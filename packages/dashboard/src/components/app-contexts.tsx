import React from 'react';
import ResourceManager from '../resource-manager';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const TooltipContext = React.createContext({
  showTooltips: true,
  toggleTooltips: () => {},
});
