import React from 'react';
import { ReportConfigProps } from 'react-components';
import clsx from 'clsx';
import { styled, useTheme } from '@material-ui/core/styles';
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
import { ExpandableMultilevelMenuProps, MultiLevelMenu } from 'react-components';
import { Menu, MenuItem } from '@material-ui/core';
import { AuthenticatorContext } from './auth-contexts';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';

import { Reports } from './report-list';
import { BuildMenuType } from './reporter-side-bar-structure';
import AllLogsReport from './reports/all-logs-report';
import DispenserStateReportConfig from './reports/dispenser-state-report';
import DoorStateReportConfig from './reports/door-state-report';
import FleetStateReportConfig from './reports/fleet-state-report';
import HealthReportConfig from './reports/health-report';
import IngestorStateReportConfig from './reports/ingestor-state-report';
import LiftStateReportConfig from './reports/lift-state-report';
import TaskSummaryReportConfig from './reports/task-summary-report';
import UserLoginFailureReportConfig from './reports/user-login-failure-report';
import UserLoginReportConfig from './reports/user-login-report';
import UserLogoutReportConfig from './reports/user-logout-report';

const drawerWidth = 240;

const classes = {
  root: 'report-dashboard-root',
  appBar: 'report-dashboard-appbar',
  appBarShift: 'report-dashboard-appbar-shift',
  menuButton: 'report-dashboard-menu-button',
  hide: 'report-dashboard-hide',
  drawer: 'report-dashboard-drawer',
  drawerPaper: 'report-dashboard-drawer-paper',
  drawerHeader: 'report-dashboard-drawer-header',
  content: 'report-dashboard-content',
  contentShift: 'report-dashboard-content-shift',
  toolbarTitle: 'report-dashboard-toolbar-title',
};

const ReportDashboardRoot = styled('div')(({ theme }) => ({
  [`&.${classes.root}`]: {
    display: 'flex',
  },
  [`& .${classes.appBar}`]: {
    transition: theme.transitions.create(['margin', 'width'], {
      easing: theme.transitions.easing.sharp,
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  [`& .${classes.appBarShift}`]: {
    width: `calc(100% - ${drawerWidth}px)`,
    marginLeft: drawerWidth,
    transition: theme.transitions.create(['margin', 'width'], {
      easing: theme.transitions.easing.easeOut,
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  [`& .${classes.menuButton}`]: {
    marginRight: theme.spacing(2),
  },
  [`& .${classes.hide}`]: {
    display: 'none',
  },
  [`& .${classes.drawer}`]: {
    width: drawerWidth,
    flexShrink: 0,
  },
  [`& .${classes.drawerPaper}`]: {
    width: drawerWidth,
  },
  [`& .${classes.drawerHeader}`]: {
    display: 'flex',
    alignItems: 'center',
    padding: theme.spacing(0, 1),
    // necessary for content to be below app bar
    ...theme.mixins.toolbar,
    justifyContent: 'flex-end',
  },
  [`& .${classes.content}`]: {
    flexGrow: 1,
    padding: theme.spacing(3),
    transition: theme.transitions.create('margin', {
      easing: theme.transitions.easing.sharp,
      duration: theme.transitions.duration.leavingScreen,
    }),
    marginLeft: -drawerWidth,
  },
  [`& .${classes.contentShift}`]: {
    transition: theme.transitions.create('margin', {
      easing: theme.transitions.easing.easeOut,
      duration: theme.transitions.duration.enteringScreen,
    }),
    marginLeft: 0,
  },
  [`& .${classes.toolbarTitle}`]: {
    flexGrow: 1,
  },
}));

export interface ReportDashboardProps {
  buildMenuReportStructure(setCurrentReport: BuildMenuType): ExpandableMultilevelMenuProps[];
}

export const ReportDashboard = (props: ReportDashboardProps) => {
  const { buildMenuReportStructure } = props;
  const theme = useTheme();
  const [open, setOpen] = React.useState(true);
  const [currentReport, setCurrentReport] = React.useState(Reports.queryAllLogs);
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);

  const [fromLogDate, setFromLogDate] = React.useState<Date>(new Date());
  const [toLogDate, setToLogDate] = React.useState<Date>(new Date());

  const handleFromLogDateChange = React.useCallback((date: any) => {
    setFromLogDate(date);
  }, []);

  const handleToLogDateChange = React.useCallback((date: any) => {
    setToLogDate(date);
  }, []);

  const itemConfig = (props: ReportConfigProps): JSX.Element | null => {
    switch (currentReport) {
      case Reports.queryAllLogs:
        return <AllLogsReport />;
      case Reports.showDispenserStateReport:
        return <DispenserStateReportConfig {...props} />;
      case Reports.showDoorStateReport:
        return <DoorStateReportConfig {...props} />;
      case Reports.showFleetStateReport:
        return <FleetStateReportConfig {...props} />;
      case Reports.showHealthReport:
        return <HealthReportConfig {...props} />;
      case Reports.showIngestorStateReport:
        return <IngestorStateReportConfig {...props} />;
      case Reports.showLiftStateReport:
        return <LiftStateReportConfig {...props} />;
      case Reports.showLoginsReport:
        return <UserLoginReportConfig {...props} />;
      case Reports.showLogoutsReport:
        return <UserLogoutReportConfig {...props} />;
      case Reports.showLoginFailuresReport:
        return <UserLoginFailureReportConfig {...props} />;
      case Reports.showTasksReport:
        return <TaskSummaryReportConfig {...props} />;
      default:
        return null;
    }
  };

  const headerTitle = React.useMemo(() => {
    switch (currentReport) {
      case Reports.queryAllLogs:
        return 'All Logs';
      case Reports.showDispenserStateReport:
        return 'Dispenser State Report';
      case Reports.showDoorStateReport:
        return 'Door State Report';
      case Reports.showFleetStateReport:
        return 'Fleet State Report';
      case Reports.showHealthReport:
        return 'Health Report';
      case Reports.showIngestorStateReport:
        return 'Ingestor State Report';
      case Reports.showLiftStateReport:
        return 'Lift State Report';
      case Reports.showLoginsReport:
        return 'Login Report';
      case Reports.showLogoutsReport:
        return 'Logout Report';
      case Reports.showLoginFailuresReport:
        return 'Login Failure Report';
      case Reports.showTasksReport:
        return 'Task Report';
      default:
        return '';
    }
  }, [currentReport]);

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
    <ReportDashboardRoot className={classes.root}>
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
            Reports - {headerTitle}
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
            <MultiLevelMenu menuStructure={buildMenuReportStructure(setCurrentReport)} />
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
    </ReportDashboardRoot>
  );
};
