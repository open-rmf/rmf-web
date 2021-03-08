import React from 'react';
import { ExpandableReporterMenuProps, ReporterMenu } from './reporter-menu';
import { buildMenu } from './reporter-side-bar';

import clsx from 'clsx';
import { makeStyles, useTheme, Theme, createStyles } from '@material-ui/core/styles';
import Drawer from '@material-ui/core/Drawer';
import CssBaseline from '@material-ui/core/CssBaseline';
import AppBar from '@material-ui/core/AppBar';
import Toolbar from '@material-ui/core/Toolbar';
import Typography from '@material-ui/core/Typography';
import Divider from '@material-ui/core/Divider';
import IconButton from '@material-ui/core/IconButton';
import MenuIcon from '@material-ui/icons/Menu';
import ChevronLeftIcon from '@material-ui/icons/ChevronLeft';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';

import { LogManagement, DoorStateReport, DoorRowsType } from 'react-components';

const drawerWidth = 240;

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    root: {
      display: 'flex',
    },
    appBar: {
      transition: theme.transitions.create(['margin', 'width'], {
        easing: theme.transitions.easing.sharp,
        duration: theme.transitions.duration.leavingScreen,
      }),
    },
    appBarShift: {
      width: `calc(100% - ${drawerWidth}px)`,
      marginLeft: drawerWidth,
      transition: theme.transitions.create(['margin', 'width'], {
        easing: theme.transitions.easing.easeOut,
        duration: theme.transitions.duration.enteringScreen,
      }),
    },
    menuButton: {
      marginRight: theme.spacing(2),
    },
    hide: {
      display: 'none',
    },
    drawer: {
      width: drawerWidth,
      flexShrink: 0,
    },
    drawerPaper: {
      width: drawerWidth,
    },
    drawerHeader: {
      display: 'flex',
      alignItems: 'center',
      padding: theme.spacing(0, 1),
      // necessary for content to be below app bar
      ...theme.mixins.toolbar,
      justifyContent: 'flex-end',
    },
    content: {
      flexGrow: 1,
      padding: theme.spacing(3),
      transition: theme.transitions.create('margin', {
        easing: theme.transitions.easing.sharp,
        duration: theme.transitions.duration.leavingScreen,
      }),
      marginLeft: -drawerWidth,
    },
    contentShift: {
      transition: theme.transitions.create('margin', {
        easing: theme.transitions.easing.easeOut,
        duration: theme.transitions.duration.enteringScreen,
      }),
      marginLeft: 0,
    },
  }),
);

const getLogLabels = () => [
  { label: 'Web Server', value: 'web-server' },
  { label: 'RMF core', value: 'rmf-core' },
];

function randomDate(start: Date, end: Date) {
  return new Date(start.getTime() + Math.random() * (end.getTime() - start.getTime()));
}

const getLogs = () => {
  const rows = [];
  for (let i = 0; i < 500; i++) {
    rows.push({
      message: 'Test' + i,
      level: 'Debug',
      timestamp: randomDate(new Date(2012, 0, 1), new Date()).toISOString(),
    });
  }
  return rows;
};

const getLogsPromise = async () => getLogs();
const getLabelsPromise = async () => getLogLabels();

export const Reporter = () => {
  const classes = useStyles();
  const theme = useTheme();
  const [open, setOpen] = React.useState(false);
  const [allLogs, setAllLogs] = React.useState(false);
  const [doorStates, setDoorStates] = React.useState(true);

  const handleDrawerOpen = () => {
    setOpen(true);
  };

  const handleDrawerClose = () => {
    setOpen(false);
  };

  const turnOffAllReports = () => {
    setAllLogs(false);
    setDoorStates(false);
  };

  return (
    <div className={classes.root}>
      <CssBaseline />
      <AppBar
        position="fixed"
        className={clsx(classes.appBar, {
          [classes.appBarShift]: open,
        })}
      >
        <Toolbar>
          <IconButton
            color="inherit"
            aria-label="open drawer"
            onClick={handleDrawerOpen}
            edge="start"
            className={clsx(classes.menuButton, open && classes.hide)}
          >
            <MenuIcon />
          </IconButton>
          <Typography variant="h6" noWrap>
            Reports
          </Typography>
        </Toolbar>
      </AppBar>
      <Drawer
        className={classes.drawer}
        variant="persistent"
        anchor="left"
        open={open}
        classes={{
          paper: classes.drawerPaper,
        }}
      >
        <div className={classes.drawerHeader}>
          <IconButton onClick={handleDrawerClose}>
            {theme.direction === 'ltr' ? <ChevronLeftIcon /> : <ChevronRightIcon />}
          </IconButton>
        </div>
        <Divider />
        <ReporterMenu
          menuStructure={
            buildMenu({
              allLogs: () => {
                turnOffAllReports();
                setAllLogs(true);
              },
              doorsStateReport: () => {
                turnOffAllReports();
                setDoorStates(true);
              },
            }) as ExpandableReporterMenuProps[]
          }
        />
      </Drawer>
      <main
        className={clsx(classes.content, {
          [classes.contentShift]: open,
        })}
      >
        <div className={classes.drawerHeader} />
        {allLogs && <LogManagement getLogs={getLogsPromise} getLabels={getLabelsPromise} />}
        {doorStates && (
          <DoorStateReport
            rows={
              [
                {
                  name: 'doorA',
                  status: 'closed',
                  message: 'test',
                  timestamp: new Date().toISOString(),
                },
              ] as DoorRowsType
            }
            tableSize={'49vh'}
          />
        )}
        {!allLogs && !doorStates && <h1>you should pick a report</h1>}
      </main>
    </div>
  );
};
