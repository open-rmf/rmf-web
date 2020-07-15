import React from 'react';
import { Redirect, Route, RouteProps } from 'react-router';
import NotFoundPage from './page-not-found';
import { LOGIN_ROUTE } from '../util/url';

interface OwnProps {
  redirectToLogin: boolean;
  isAuthenticated: boolean;
}

type Props = RouteProps & OwnProps;

/**
 * This component validates if the user is authenticated before rendering component passed as a prop.
 * @param redirectToLogin - redirects to login route if the user not authenticated.
 * @param isAuthenticated - true if the user isAuthenticated.
 */
const PrivateRoute = (props: Props): React.ReactElement => {
  const { component, isAuthenticated, redirectToLogin, ...rest } = props;
  const Component = component as React.ComponentClass<any>;
  const renderRoute = (props: any) => {
    if (isAuthenticated) {
      return <Component {...props} />;
    }
    if (redirectToLogin) {
      return <Redirect to={LOGIN_ROUTE} />;
    } else {
      return <NotFoundPage />;
    }
  };
  return <Route {...rest} render={renderRoute} />;
};

export default PrivateRoute;
