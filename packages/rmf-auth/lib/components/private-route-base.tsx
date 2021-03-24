import Debug from 'debug';
import React from 'react';
import { RouteProps } from 'react-router';
import { User } from '../user';

const debug = Debug('PrivateRoute');

export interface PrivateRouteBaseProps extends React.PropsWithChildren<RouteProps> {
  // if true, do not redirect to login url if not authenticated
  noRedirectToLogin?: boolean;
  loginRoute?: string;
  unauthorized: React.ReactElement;
  user?: User | null;
  redirect: React.ReactElement;
}

/**
 * This component validates if the user is authenticated before rendering component passed as a
 * prop.
 */
export const PrivateRouteBase = ({
  noRedirectToLogin,
  children,
  loginRoute,
  unauthorized,
  redirect,
  user,
  ...rest
}: PrivateRouteBaseProps): React.ReactElement | null => {
  // const user = React.useContext(UserContext);
  // const location = useLocation();

  return user
    ? (children as React.ReactElement)
    : !noRedirectToLogin
    ? (redirect as React.ReactElement)
    : (unauthorized as React.ReactElement);

  // function render(): JSX.Element | null {
  //   if (user) {
  //     return children;
  //   } else {
  //     if (!noRedirectToLogin) {
  //       debug('accessing private route while unauthenticated');
  //       debug('redirecting to login page');
  //       return { redirect };
  //     } else {
  //       return { unauthorized };
  //     }
  //   }
  // }

  // return render();
};

export default PrivateRouteBase;
