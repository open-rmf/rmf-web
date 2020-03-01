import React from 'react';
import './App.css';
import ScheduleVisualizer from './components/schedule-visualizer';
import {
  AppBar,
  IconButton,
  Popper,
  Toolbar,
  Typography,
  makeStyles,
} from '@material-ui/core/';
import ArrowDropDownIcon from '@material-ui/icons/ArrowDropDown';
import 'typeface-roboto';
import MainMenu from './main-menu';

const useStyles = makeStyles(() => ({
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
    float: 'right',
    top: 100,
    left: '80%',
    zIndex: 2,
  },
}));

export function App(props: any) {
  const classes = useStyles();
  const [showOmniPanel, setShowOmniPanel] = React.useState(false);
  let test = React.createRef<HTMLDivElement>();

  return (
    <div className={classes.container} ref={test}>
      <AppBar position="static">
        <Toolbar>
          <Typography variant="h6" className={classes.toolBarTitle}>
            Dashboard
          </Typography>
          <IconButton color="inherit" onClick={() => setShowOmniPanel(true)}>
            <ArrowDropDownIcon></ArrowDropDownIcon>
          </IconButton>
        </Toolbar>
      </AppBar>
      <ScheduleVisualizer />
      <div className={classes.omniPanel}>
        <MainMenu />
      </div>
    </div>
  );
}

export default App;
