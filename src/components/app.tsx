import { AppBar, Fade, IconButton, makeStyles, Toolbar, Typography } from '@material-ui/core/';
import { Dashboard as DashboardIcon, Settings as SettingsIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import appConfig from '../app-config';
import { LOGIN_ROUTE } from '../util/url';
import './app.css';
import { AuthenticatorContext, UserContext } from './auth/contexts';
import Login from './auth/login';
import PrivateRoute from './auth/private-route';
import { User } from './auth/user';
import Dashboard from './dashboard';
import NotFoundPage from './error-pages/page-not-found';

export default function App(): React.ReactElement {
  const [authInitialized, setAuthInitialized] = React.useState(false);
  const [user, setUser] = React.useState<User | null>(null);
  const authenticator = appConfig.authenticator;

  React.useEffect(() => {
    setLoading({ caption: 'Connecting to SOSS server...' });
    transportFactory()
      .then(x => {
        x.on('error', console.error);
        x.once('close', () => {
          setLoading({ caption: 'Lost connection to SOSS', variant: 'error' });
          setTransport(undefined);
        });
        doorStateManager.startSubscription(x);
        dispenserStateManager.startSubscription(x);
        liftStateManager.startSubscription(x);
        fleetManager.startSubscription(x);

        fleetManager.on('updated', () => setFleets(fleetManager.fleets()));
        liftStateManager.on('updated', () => setLiftStates(liftStateManager.liftStates()));
        doorStateManager.on('updated', () => setDoorStates(doorStateManager.doorStates()));
        dispenserStateManager.on('updated', () =>
          setDispenserStates(dispenserStateManager.dispenserStates()),
        );
        setTransport(x);
      })
      .catch((e: CloseEvent) => {
        setLoading({ caption: `Unable to connect to SOSS server (${e.code})`, variant: 'error' });
      });
  }, [transportFactory, doorStateManager, liftStateManager, dispenserStateManager, fleetManager]);

  React.useEffect(() => {
    if (!transport) {
      return;
    }
    setLoading({ caption: 'Downloading building map...' });
    const request = new RomiCore.GetBuildingMap_Request();
    transport
      .call(RomiCore.getBuildingMap, request)
      .then(result => {
        setBuildingMap(result.building_map);
        setLoading(null);
      })
      .catch(() => {
        setLoading({ caption: 'Unable to download building map', variant: 'error' });
      });
  }, [transport]);

  React.useEffect(() => {
    if (!trajectoryManagerFactory) {
      return;
    }
    (async () => {
      authenticator.on('userChanged', newUser => setUser(newUser));
      await authenticator.init();
      setUser(authenticator.user || null);
      setAuthInitialized(true);
    })();
  }, [authenticator]);

  return authInitialized ? (
    <AuthenticatorContext.Provider value={authenticator}>
      <UserContext.Provider value={user}>
        <BrowserRouter>
          <Switch>
            <Route exact={true} path={LOGIN_ROUTE}>
              <Login />
            </Route>
            <PrivateRoute exact={true} path={'/'}>
              <Dashboard />
            </PrivateRoute>
            <Route component={NotFoundPage} />
          </Switch>
        </BrowserRouter>
      </UserContext.Provider>
    </AuthenticatorContext.Provider>
  ) : (
    <></>
  );
}
