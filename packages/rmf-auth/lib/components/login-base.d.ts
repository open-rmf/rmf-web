/// <reference types="react" />
import { Authenticator } from '../authenticator';
export interface LoginBaseProps {
  /**
   * defaults to url of `DASHBOARD_ROUTE`
   */
  title: string;
  successRedirectUri: string;
  authenticator: Authenticator;
}
export declare const LoginBase: (props: LoginBaseProps) => JSX.Element;
export default LoginBase;
