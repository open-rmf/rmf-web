import React from 'react';
import { BuildMenuType } from './reporter-side-bar-structure';
import { ReportConfigProps } from 'react-components';

import clsx from 'clsx';
import { makeStyles, useTheme, Theme } from '@material-ui/core/styles';
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
import { MaterialUiPickersDate } from '@material-ui/pickers/typings/date';

import { ReportContainer, Reports } from './report-list';
import { ExpandableMultilevelMenuProps, MultiLevelMenu } from 'react-components';
import { Menu, MenuItem } from '@material-ui/core';
import { AuthenticatorContext } from './auth-contexts';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';

const drawerWidth = 240;

const useStyles = makeStyles((theme: Theme) => ({
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
  toolbarTitle: {
    flexGrow: 1,
  },
}));

export interface ReportDashboardProps {
  buildMenuReportStructure(setCurrentReport: BuildMenuType): ExpandableMultilevelMenuProps[];
  reportContainer: typeof ReportContainer;
}

export const ReportDashboard = (props: ReportDashboardProps) => {
  const { buildMenuReportStructure, reportContainer } = props;
  const classes = useStyles();
  const theme = useTheme();
  const [open, setOpen] = React.useState(true);
  const [currentReport, setCurrentReport] = React.useState(Reports.queryAllLogs);
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);

  const [fromLogDate, setFromLogDate] = React.useState<MaterialUiPickersDate>(new Date());
  const [toLogDate, setToLogDate] = React.useState<MaterialUiPickersDate>(new Date());

  const [header, setHeader] = React.useState<string>('');

  const handleFromLogDateChange = React.useCallback((date: MaterialUiPickersDate) => {
    setFromLogDate(date);
  }, []);

  const handleToLogDateChange = React.useCallback((date: MaterialUiPickersDate) => {
    setToLogDate(date);
  }, []);

  const itemConfig = (props: ReportConfigProps): JSX.Element | null => {
    if (Object.prototype.hasOwnProperty.call(reportContainer, currentReport)) {
      switch (currentReport) {
        case Reports.queryAllLogs:
          const QueryConfig = reportContainer[currentReport];
          return <QueryConfig />;
        case Reports.showDispenserStateReport:
        case Reports.showDoorStateReport:
        case Reports.showFleetStateReport:
        case Reports.showHealthReport:
        case Reports.showIngestorStateReport:
        case Reports.showLiftStateReport:
        case Reports.showLoginsReport:
        case Reports.showLogoutsReport:
        case Reports.showLoginFailuresReport:
        case Reports.showTasksReport:
          const Config = reportContainer[currentReport];
          return <Config {...props} />;
        default:
          return null;
      }
    } else {
      return null;
    }
  };

  const headerTitle = (): void => {
    switch (currentReport) {
      case Reports.queryAllLogs:
        setHeader(currentReport.split('query')[1]);
        break;
      case Reports.showDispenserStateReport:
      case Reports.showDoorStateReport:
      case Reports.showFleetStateReport:
      case Reports.showHealthReport:
      case Reports.showIngestorStateReport:
      case Reports.showLiftStateReport:
      case Reports.showLoginsReport:
      case Reports.showLogoutsReport:
      case Reports.showLoginFailuresReport:
      case Reports.showTasksReport:
        setHeader(currentReport.split('show')[1].split('Report')[0]);
        break;
      default:
        break;
    }
  };

  React.useEffect(() => {
    headerTitle();
  });

  const setReport = React.useCallback(
    (report: Reports) => {
      setCurrentReport(report);
    },
    [setCurrentReport],
  );

  const handleDrawerOpen = () => {
    setOpen(true);
  };

  const handleDrawerClose = () => {
    setOpen(false);
  };

  const authenticator = React.useContext(AuthenticatorContext);

  async function handleLogout(): Promise<void> {
    try {
      await authenticator.logout();
    } catch (e) {
      console.error(`error logging out: ${e.message}`);
    }
  }

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
          <Typography variant="h6" className={classes.toolbarTitle}>
            Reports - {header}
          </Typography>
          {authenticator.user && (
            <>
              <IconButton
                id="user-btn"
                aria-label={'user-btn'}
                color="inherit"
                onClick={(event) => setAnchorEl(event.currentTarget)}
              >
                <AccountCircleIcon />
              </IconButton>
              <Menu
                anchorEl={anchorEl}
                getContentAnchorEl={null}
                anchorOrigin={{
                  vertical: 'bottom',
                  horizontal: 'right',
                }}
                transformOrigin={{
                  vertical: 'top',
                  horizontal: 'right',
                }}
                open={!!anchorEl}
                onClose={() => setAnchorEl(null)}
              >
                <MenuItem id="logout-btn" onClick={handleLogout}>
                  Logout
                </MenuItem>
              </Menu>
            </>
          )}
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
        {open && (
          <>
            <div className={classes.drawerHeader}>
              <IconButton onClick={handleDrawerClose} aria-label="close drawer">
                {theme.direction === 'ltr' ? <ChevronLeftIcon /> : <ChevronRightIcon />}
              </IconButton>
            </div>

            <Divider />
            <MultiLevelMenu menuStructure={buildMenuReportStructure(setReport)} />
          </>
        )}
      </Drawer>
      <main
        className={clsx(classes.content, {
          [classes.contentShift]: open,
        })}
      >
        <div className={classes.drawerHeader} />
        {itemConfig({
          fromLogDate,
          toLogDate,
          onSelectFromDate: handleFromLogDateChange,
          onSelectToDate: handleToLogDateChange,
        })}
      </main>
    </div>
  );
};
