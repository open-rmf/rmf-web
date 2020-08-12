import React from 'react';
import Authenticator from './authenticator';

export const AuthContext = React.createContext<Authenticator | null>(null);

export default AuthContext;
