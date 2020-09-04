import Debug from 'debug';
import React from 'react';
import { Redirect, Route, RouteProps, useLocation } from 'react-router';
import { UserContext } from '../../app-contexts';
import { LOGIN_ROUTE } from '../../util/url';
import Unauthorized from '../error-pages/unauthorized';

const debug = Debug('PrivateRoute');

interface Props extends React.PropsWithChildren<RouteProps> {
  // if true, do not redirect to login url if not authenticated
  noRedirectToLogin?: boolean;
}

/**
 * This component validates if the user is authenticated before rendering component passed as a
 * prop.
 */
const PrivateRoute = ({ noRedirectToLogin, children, ...rest }: Props): React.ReactElement => {
  const user = React.useContext(UserContext);
  const location = useLocation();

  function render(): React.ReactNode {
    if (user) {
      return children;
    } else {
      if (!noRedirectToLogin) {
        debug('accessing private route while unauthenticated');
        debug('redirecting to login page');
        return <Redirect to={{ pathname: LOGIN_ROUTE, state: { from: location } }} />;
      } else {
        return <Unauthorized />;
      }
    }
  }

  return <Route {...rest}>{render()}</Route>;
};

export default PrivateRoute;
