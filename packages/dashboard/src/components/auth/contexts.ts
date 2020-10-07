import React from 'react';
import appConfig from '../../app-config';
import { User } from './user';

export const AuthenticatorContext = React.createContext(appConfig.authenticator);
export const UserContext = React.createContext<User | null>(null);
