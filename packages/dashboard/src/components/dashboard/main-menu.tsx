import { Divider, makeStyles, Typography, Button } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { TooltipsContext } from '../app-contexts';
import { OmniPanelViewIndex } from './dashboard';
import {
  Notification,
  SystemSummaryItemState,
  DashboardTooltip,
  SystemSummaryAlert,
  SystemSummaryTaskState,
  SystemSummaryBanner,
  SystemSummarySpoiltItems,
} from 'react-components';
import { RmfHealthContext } from '../rmf-app';
import { HealthStatus } from '../../managers/rmf-health-state-manager';

const debug = Debug('MainMenu');

const useStyles = makeStyles((theme) => ({
  root: {
    padding: theme.spacing(2),
    backgroundColor: theme.palette.background.paper,
  },
  buttons: {
    width: '100%',
    margin: '0.5rem 0',
  },
  commandButton: {
    backgroundColor: theme.palette.warning.dark,
    color: 'white',
  },
  divider: {
    margin: '0.5rem 0',
  },
  systemSummaryHeader: {
    margin: '0.5rem 0',
  },
}));

export interface MainMenuProps {
  pushView(view: OmniPanelViewIndex): void;
  tasks: RomiCore.TaskSummary[];
  notifications: Notification[];
  spoiltDoorClick?(door: RomiCore.Door): void;
  spoiltLiftClick?(lift: RomiCore.Lift): void;
  spoiltRobotClick?(fleet: string, robot: RomiCore.RobotState): void;
  spoiltDispenserClick?(event: React.MouseEvent, guid: string): void;
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  const { showTooltips } = React.useContext(TooltipsContext);
  const {
    pushView,
    tasks,
    notifications,
    spoiltDispenserClick,
    spoiltDoorClick,
    spoiltLiftClick,
    spoiltRobotClick,
  } = props;
  debug('render');
  const classes = useStyles();

  const handleMainMenuDoorsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Doors);
  }, [pushView]);

  const handleMainMenuLiftsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Lifts);
  }, [pushView]);

  const handleMainMenuRobotsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Robots);
  }, [pushView]);

  const handleMainMenuDispensersClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Dispensers);
  }, [pushView]);

  const handleMainMenuCommandsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Commands);
  }, [pushView]);

  const handleMainMenuNegotiationsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Negotiations);
  }, [pushView]);

  const handleMainMenuTasksClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Tasks);
  }, [pushView]);

  const bannerIsError = (healthStatus: HealthStatus): boolean => {
    let doorCount = 0;
    let liftCount = 0;
    let dispenserCount = 0;
    let robotCount = 0;

    if (healthStatus.robot.robotSummary) {
      robotCount = healthStatus.robot.robotSummary.outOfOrder;
    }

    if (healthStatus.door.itemSummary) {
      doorCount = healthStatus.door.itemSummary.outOfOrder;
    }

    if (healthStatus.lift.itemSummary) {
      liftCount = healthStatus.lift.itemSummary.outOfOrder;
    }

    if (healthStatus.dispenser.itemSummary) {
      dispenserCount = healthStatus.dispenser.itemSummary.outOfOrder;
    }

    return doorCount + liftCount + dispenserCount + robotCount !== 0;
  };

  const healthStatus = React.useContext(RmfHealthContext);

  return (
    <React.Fragment>
      <SystemSummaryBanner imageSrc={'/favicon.ico'} isError={bannerIsError(healthStatus)} />
      <div className={classes.root}>
        <SystemSummaryAlert notifications={notifications} />
        <Divider className={classes.divider} />

        <SystemSummarySpoiltItems
          doors={healthStatus.door.spoiltDoors}
          lifts={healthStatus.lift.spoiltLifts}
          dispensers={healthStatus.dispenser.spoiltDispensers}
          robots={healthStatus.robot.spoiltRobots}
          spoiltDoorClick={spoiltDoorClick}
          spoiltDispenserClick={spoiltDispenserClick}
          spoiltLiftClick={spoiltLiftClick}
          spoiltRobotClick={spoiltRobotClick}
        />
        <Divider className={classes.divider} />

        <Typography variant="h5" className={classes.systemSummaryHeader}>
          Systems Summary
        </Typography>

        <SystemSummaryItemState
          itemSummary={healthStatus.door}
          onClick={handleMainMenuDoorsClick}
        />
        <SystemSummaryItemState
          itemSummary={healthStatus.lift}
          onClick={handleMainMenuLiftsClick}
        />
        <SystemSummaryItemState
          itemSummary={healthStatus.dispenser}
          onClick={handleMainMenuDispensersClick}
        />
        <SystemSummaryItemState
          itemSummary={healthStatus.robot}
          onClick={handleMainMenuRobotsClick}
        />
        <Divider className={classes.divider} />

        <Typography variant="h6">Task Statuses</Typography>
        <SystemSummaryTaskState tasks={tasks} onClick={handleMainMenuTasksClick} />
        <Button
          color="primary"
          className={classes.buttons}
          variant="contained"
          onClick={handleMainMenuNegotiationsClick}
        >
          <DashboardTooltip
            title="This panel shows the negotiations between robots when there are conflicts in trajectories"
            id="negotiations-tooltip"
            enabled={showTooltips}
          >
            <Typography variant="body1">Negotiations</Typography>
          </DashboardTooltip>
        </Button>
        <Button
          color="primary"
          className={classes.buttons}
          variant="contained"
          onClick={handleMainMenuTasksClick}
        >
          <Typography variant="body1">Plans</Typography>
        </Button>
        <Divider className={classes.divider} />

        <Typography variant="h6">Command Center</Typography>
        <Button
          className={`${classes.buttons} ${classes.commandButton}`}
          variant="contained"
          onClick={handleMainMenuCommandsClick}
        >
          <DashboardTooltip
            title="This panel shows the commands that a user can request and RoMi will allocate the most suitable robot for the task"
            id="commands-tooltip"
            enabled={showTooltips}
          >
            <Typography variant="body1">Commands</Typography>
          </DashboardTooltip>
        </Button>
      </div>
    </React.Fragment>
  );
});

export default MainMenu;
