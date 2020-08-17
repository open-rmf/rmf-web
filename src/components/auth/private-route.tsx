import debug from 'debug';
import React from 'react';
import { Redirect, Route, RouteProps, useLocation } from 'react-router';
import { UserContext } from '../../app-contexts';
import { LOGIN_ROUTE } from '../../util/url';
import Unauthorized from './unauthorized';

interface Props extends RouteProps {
  // if true, do not redirect to login url if not authenticated
  noRedirect?: boolean;
  children: React.ReactNode;
}

/**
 * This component validates if the user is authenticated before rendering component passed as a
 * prop.
 */
const PrivateRoute = ({ noRedirect, children, ...rest }: Props): React.ReactElement => {
  const user = React.useContext(UserContext);
  const location = useLocation();

  function render(): React.ReactNode {
    if (user) {
      return children;
    } else {
      if (!noRedirect) {
        debug.log('accessing private route while unauthenticated');
        debug.log('redirecting to login page');
        return <Redirect to={{ pathname: LOGIN_ROUTE, state: { from: location } }} />;
      } else {
        return <Unauthorized />;
      }
    }
  }

  return <Route {...rest}>{render()}</Route>;
};

export default PrivateRoute;
