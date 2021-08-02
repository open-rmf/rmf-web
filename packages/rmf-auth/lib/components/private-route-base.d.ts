import React from 'react';
import { RouteProps } from 'react-router-dom';
export interface PrivateRouteBaseProps extends React.PropsWithChildren<RouteProps> {
  noRedirectToLogin?: boolean;
  unauthorized?: React.ReactElement | null;
  user?: string | null;
  redirect: React.ReactElement;
}
/**
 * This component validates if the user is authenticated before rendering component passed as a
 * prop.
 */
export declare const PrivateRouteBase: ({
  noRedirectToLogin,
  children,
  unauthorized,
  redirect,
  user,
}: PrivateRouteBaseProps) => React.ReactElement | null;
export default PrivateRouteBase;
