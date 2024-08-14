import React from 'react';
import { RouteProps } from 'react-router-dom';

export interface PrivateRouteProps {
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
}: React.PropsWithChildren<PrivateRouteProps & RouteProps>): JSX.Element => {
  return <>{user ? children : unauthorizedComponent}</>;
};

export default PrivateRoute;
