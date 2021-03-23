import Debug from 'debug';
import React from 'react';
import { Redirect, Route, RouteProps, useLocation } from 'react-router';
import { User } from '../user';
// import Unauthorized from '../error-pages/unauthorized';
// import { UserContext } from '../contexts';

const debug = Debug('PrivateRoute');

export interface PrivateRouteProps extends React.PropsWithChildren<RouteProps> {
  // if true, do not redirect to login url if not authenticated
  noRedirectToLogin?: boolean;
  loginRoute?: string;
  unauthorized: React.ReactElement;
  user?: User | null;
}

/**
 * This component validates if the user is authenticated before rendering component passed as a
 * prop.
 */
const PrivateRoute = ({
  noRedirectToLogin,
  children,
  loginRoute,
  unauthorized,
  user,
  ...rest
}: PrivateRouteProps): JSX.Element | null => {
  // const user = React.useContext(UserContext);
  const location = useLocation();

  function render() {
    if (user) {
      return children;
    } else {
      if (!noRedirectToLogin) {
        debug('accessing private route while unauthenticated');
        debug('redirecting to login page');
        return <Redirect to={{ pathname: loginRoute, state: { from: location } }} />;
      } else {
        return { unauthorized };
      }
    }
  }

  return <Route {...rest}>{render()}</Route>;
};

export default PrivateRoute;
