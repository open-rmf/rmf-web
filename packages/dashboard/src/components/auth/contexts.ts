import { Permission, User } from 'api-client';
import React from 'react';
import appConfig from '../../app-config';

export interface UserProfile {
  user: User;
  permissions: Permission[];
}

export const AuthenticatorContext = React.createContext(appConfig.authenticator);
export const UserProfileContext = React.createContext<UserProfile | null>(null);
