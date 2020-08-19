import React from 'react';
import { User } from './components/auth/user';
import { ResourceConfigurationsType } from './resource-manager';

/* Declares the UserContext which contains the currently logged in user, so
that any component may use if without having to pass the prop around */
export const UserContext = React.createContext<User | null>(null);

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceConfigurationsType>({});
