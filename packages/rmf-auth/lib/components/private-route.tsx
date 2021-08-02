import React from 'react';
import { Route, RouteProps } from 'react-router-dom';

export interface PrivateRouteProps extends React.PropsWithChildren<RouteProps> {
  user: string | null;
  /**
   * Component to render if `user` is undefined.
   */
  unauthorizedComponent?: React.ReactNode;
}

export const PrivateRoute = ({
  user,
  unauthorizedComponent = 'Unauthorized',
  children,
  ...rest
}: PrivateRouteProps): JSX.Element => {
  return <Route {...rest}>{user ? children : unauthorizedComponent}</Route>;
};

export default PrivateRoute;
