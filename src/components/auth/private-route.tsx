import React from 'react';
import { Redirect, Route, RouteProps, useLocation } from 'react-router';
import { AuthContext } from './context';
import Unauthorized from './unauthorized';
import { LOGIN_ROUTE } from '../../util/url';

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
  const auth = React.useContext(AuthContext);
  const location = useLocation();
  return (
    <Route {...rest}>
      {auth && auth.authenticated ? (
        children
      ) : !noRedirect ? (
        <Redirect to={{ pathname: LOGIN_ROUTE, state: { from: location } }} />
      ) : (
        <Unauthorized />
      )}
    </Route>
  );
};

export default PrivateRoute;
