import { AppBar, IconButton, makeStyles, Toolbar, Typography } from '@material-ui/core/';
import { Dashboard as DashboardIcon } from '@material-ui/icons';
import React from 'react';
import 'typeface-roboto';
import ScheduleVisualizer from '../components/schedule-visualizer';
import OmniPanel, { OmniPanelView } from '../omni-panel';
import buildingMap from './data/building-map';
import doorStates from './data/door-states';
import fleets from './data/fleets';
import liftStates from './data/lift-states';

const borderRadius = 20;

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

export default function App() {
  const classes = useStyles();
  const [showOmniPanel, setShowOmniPanel] = React.useState(true);

  return (
    <React.Fragment>
      <div className={classes.container}>
        <AppBar position="static">
          <Toolbar>
            <Typography variant="h6" className={classes.toolBarTitle}>
              Dashboard
            </Typography>
            <IconButton color="inherit" onClick={() => setShowOmniPanel(true)}>
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
    </React.Fragment>
  );
}
