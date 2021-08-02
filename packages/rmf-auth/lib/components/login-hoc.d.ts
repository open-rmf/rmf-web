/// <reference types="react" />
import { Authenticator } from '..';
import { Redirect } from 'react-router-dom';
export interface LoginProps {
  user: string | null;
  authenticator: Authenticator;
  successRedirectUri: string;
  title: string;
}
export declare const LoginHOC: (
  RedirectComponent: typeof Redirect,
) => (props: LoginProps) => JSX.Element;
