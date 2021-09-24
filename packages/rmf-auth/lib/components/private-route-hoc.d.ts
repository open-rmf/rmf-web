import React from 'react';
import { Redirect, Route, RouteProps, useLocation } from 'react-router-dom';
export interface PrivateRouteProps extends React.PropsWithChildren<RouteProps> {
  noRedirectToLogin?: boolean;
  redirectPath: string;
  unauthorized?: React.ReactElement | null;
  user: string | null;
}
export declare const PrivateRouteHOC: (
  RouteComponent: typeof Route,
  RedirectComponent: typeof Redirect,
  hookUseLocation: typeof useLocation,
) => ({
  noRedirectToLogin,
  children,
  redirectPath,
  unauthorized,
  user,
  ...rest
}: PrivateRouteProps) => JSX.Element;
export default PrivateRouteHOC;
