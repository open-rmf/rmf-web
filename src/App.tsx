import {
  AppBar,
  IconButton,
  makeStyles,
  Toolbar,
  Typography,
} from '@material-ui/core/';
import { Dashboard as DashboardIcon } from '@material-ui/icons';
import React from 'react';
import {
  BrowserRouter as Router,
  Redirect,
  Route,
  Switch,
} from 'react-router-dom';
import 'typeface-roboto';
import './App.css';
import ScheduleVisualizer from './components/schedule-visualizer';
import Login from './login';
import buildingMap from './mock/data/building-map';
import doorStates from './mock/data/door-states';
import fleets from './mock/data/fleets';
import liftStates from './mock/data/lift-states';
import OmniPanel, { OmniPanelView } from './omni-panel';
import { AuthService } from './auth-service';

const borderRadius = 20;

// const transport = new SossTransport('romi-dashboard', 'url', 'token');

const useStyles = makeStyles(theme => ({
  container: {
    display: 'flex',
    flexFlow: 'column',
    height: '100%',
  },
  toolBarTitle: {
    flexGrow: 1,
  },
  omniPanel: {
    position: 'fixed',
    width: 350,
    top: 100,
    right: '2%',
    bottom: '3%',
    backgroundColor: theme.palette.background.default,
    zIndex: 1024,
    borderTopLeftRadius: borderRadius,
    borderTopRightRadius: borderRadius,
    boxShadow: theme.shadows[12],
  },
  topLeftBorder: {
    borderTopLeftRadius: borderRadius,
  },
  topRightBorder: {
    borderTopRightRadius: borderRadius,
  },
}));

const auth = new AuthService('https://localhost:8001');

// A wrapper for <Route> that redirects to the login
// screen if you're not yet authenticated.
function PrivateRoute({ children, ...rest }: any) {
  return (
    <Route
      {...rest}
      render={({ location }) =>
        auth.isAuthenticated ? (
          children
        ) : (
          <Redirect
            to={{
              pathname: '/login',
              state: { from: location },
            }}
          />
        )
      }
    />
  );
}

export function App(props: any) {
  const classes = useStyles();
  const [showOmniPanel, setShowOmniPanel] = React.useState(true);

  return (
    <Router>
      <Switch>
        <Route path="/login">
          <Login auth={auth} />
        </Route>
        <PrivateRoute path="">
          <div className={classes.container}>
            <AppBar position="static">
              <Toolbar>
                <Typography variant="h6" className={classes.toolBarTitle}>
                  Dashboard
                </Typography>
                <IconButton
                  color="inherit"
                  onClick={() => setShowOmniPanel(true)}
                >
                  <DashboardIcon />
                </IconButton>
              </Toolbar>
            </AppBar>
            <ScheduleVisualizer />
            {showOmniPanel && (
              <OmniPanel
                className={classes.omniPanel}
                classes={{
                  backButton: classes.topLeftBorder,
                  closeButton: classes.topRightBorder,
                }}
                buildingMap={buildingMap}
                doorStates={doorStates}
                liftStates={liftStates}
                fleets={fleets}
                initialView={OmniPanelView.MainMenu}
                onClose={() => setShowOmniPanel(false)}
              />
            )}
          </div>
        </PrivateRoute>
      </Switch>
    </Router>
  );
}

export default App;
