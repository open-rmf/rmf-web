import React from 'react';
import { User } from 'rmf-auth';
import appConfig from '../../app-config';

export const AuthenticatorContext = React.createContext(appConfig.authenticator);
export const UserContext = React.createContext<User | null>(null);
