import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { createContext } from 'react';
import { ResourceConfigurationsType } from './resource-manager';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = createContext<ResourceConfigurationsType>({});

export const DoorStateContext = createContext<Record<string, RomiCore.DoorState>>({});
