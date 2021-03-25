import React from 'react';
import { Redirect, Route, RouteProps, useLocation } from 'react-router';
import { LOGIN_ROUTE } from '../../util/url';
import { UserContext } from './contexts';
import { Unauthorized } from 'react-components';
import { PrivateRouteBase } from 'rmf-auth';

export interface PrivateRouteProps extends React.PropsWithChildren<RouteProps> {
  // if true, do not redirect to login url if not authenticated
  noRedirectToLogin?: boolean;
}

/**
 * This component validates if the user is authenticated before rendering component passed as a
 * prop.
 */
export const PrivateRoute = ({
  noRedirectToLogin,
  children,
  ...rest
}: PrivateRouteProps): JSX.Element | null => {
  const user = React.useContext(UserContext);
  const location = useLocation();

  return (
    <Route {...rest}>
      <PrivateRouteBase
        user={user}
        noRedirectToLogin={noRedirectToLogin}
        unauthorized={<Unauthorized />}
        redirect={<Redirect to={{ pathname: LOGIN_ROUTE, state: { from: location } }} />}
      >
        {children}
      </PrivateRouteBase>
    </Route>
  );
};

export default PrivateRoute;
