import {
  AppBar,
  IconButton,
  makeStyles,
  Toolbar,
  Typography,
} from '@material-ui/core/';
import { Dashboard as DashboardIcon } from '@material-ui/icons';
import React from 'react';
import 'typeface-roboto';
import './App.css';
import ScheduleVisualizer from './components/schedule-visualizer';
import buildingMap from './mock/data/building-map';
import doorStates from './mock/data/door-states';
import fleets from './mock/data/fleets';
import liftStates from './mock/data/lift-states';
import OmniPanel, { OmniPanelView } from './omni-panel';

const BorderRadius = 20;

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
    zIndex: 2,
    borderTopLeftRadius: BorderRadius,
    borderTopRightRadius: BorderRadius,
    boxShadow: theme.shadows[12],
  },
  topLeftBorder: {
    borderTopLeftRadius: BorderRadius,
  },
  topRightBorder: {
    borderTopRightRadius: BorderRadius,
  },
}));

export function App(props: any) {
  const classes = useStyles();
  const [showOmniPanel, setShowOmniPanel] = React.useState(true);

  return (
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
  );
}

export default App;
