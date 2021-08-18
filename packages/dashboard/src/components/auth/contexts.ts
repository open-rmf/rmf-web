import { Permission, User } from 'api-client';
import React from 'react';

export interface UserProfile {
  user: User;
  permissions: Permission[];
}

export const UserProfileContext = React.createContext<UserProfile | null>(null);
