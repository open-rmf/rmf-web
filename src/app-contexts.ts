import { createContext } from 'react';
import { ResourceConfigurationsType } from './resource-manager';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = createContext<ResourceConfigurationsType>({});
