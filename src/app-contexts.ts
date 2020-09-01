import { createContext } from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { ResourceConfigurationsType } from './resource-manager';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = createContext<ResourceConfigurationsType>({});

export const DispenserStateContext = createContext<
  Readonly<Record<string, RomiCore.DispenserState>>
>({});
