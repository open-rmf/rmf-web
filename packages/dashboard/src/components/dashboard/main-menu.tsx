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
  ItemSummary,
  SystemSummaryBanner,
  liftModeToString,
  robotModeToString,
  SystemSummarySpoiltItems,
  SpoiltItem,
} from 'react-components';

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

export interface ItemState {
  doors: Record<string, RomiCore.DoorState>;
  lifts: Record<string, RomiCore.LiftState>;
  robots: Record<string, RomiCore.FleetState>;
  dispensers: Record<string, RomiCore.DispenserState>;
}

export interface MainMenuProps {
  pushView(view: OmniPanelViewIndex): void;
  itemState: ItemState;
  tasks: RomiCore.TaskSummary[];
  notifications: Notification[];
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  const { showTooltips } = React.useContext(TooltipsContext);
  const { pushView, itemState, tasks, notifications } = props;
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

  const getDoorSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0 };
    const spoiltEquipment: string[] = [];
    const doors = itemState.doors;
    const doorKeys = Object.keys(doors);

    doorKeys.forEach((door) => {
      switch (doors[door].current_mode.value) {
        case RomiCore.DoorMode.MODE_CLOSED:
        case RomiCore.DoorMode.MODE_MOVING:
        case RomiCore.DoorMode.MODE_OPEN:
          modeCounter.operational += 1;
          break;
        default:
          modeCounter.outOfOrder += 1;
          spoiltEquipment.push(door + ' - Unknown');
          break;
      }
    });
    return {
      item: 'Door',
      summary: { operational: modeCounter.operational, outOfOrder: modeCounter.outOfOrder },
      outOfOrder: spoiltEquipment,
    };
  };

  const getLiftSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0 };
    const spoiltEquipment: string[] = [];
    const lifts = itemState.lifts;
    const liftKeys = Object.keys(lifts);

    liftKeys.forEach((lift) => {
      switch (lifts[lift].current_mode) {
        case RomiCore.LiftState.MODE_HUMAN:
        case RomiCore.LiftState.MODE_AGV:
        case RomiCore.LiftState.MODE_OFFLINE:
          modeCounter.operational += 1;
          break;
        case RomiCore.LiftState.MODE_FIRE:
        case RomiCore.LiftState.MODE_EMERGENCY:
        case RomiCore.LiftState.MODE_UNKNOWN:
          spoiltEquipment.push(lift + ` - ${liftModeToString(lifts[lift].current_mode)}`);
          modeCounter.outOfOrder += 1;
          break;
      }
    });
    return {
      item: 'Lift',
      summary: { operational: modeCounter.operational, outOfOrder: modeCounter.outOfOrder },
      outOfOrder: spoiltEquipment,
    };
  };

  const getDispenserSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0 };
    const spoiltEquipment: string[] = [];
    const dispensers = itemState.dispensers;
    const dispenserKeys = Object.keys(dispensers);

    dispenserKeys.forEach((dispenser) => {
      switch (dispensers[dispenser].mode) {
        case RomiCore.DispenserState.IDLE:
        case RomiCore.DispenserState.BUSY:
        case RomiCore.DispenserState.OFFLINE:
          modeCounter.operational += 1;
          break;
        default:
          spoiltEquipment.push(dispenser + '- Unknown');
          modeCounter.outOfOrder += 1;
          break;
      }
    });

    return {
      item: 'Dispensers',
      summary: { operational: modeCounter.operational, outOfOrder: modeCounter.outOfOrder },
      outOfOrder: spoiltEquipment,
    };
  };

  const getRobotSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0, charging: 0, idle: 0 };
    const spoiltEquipment: string[] = [];
    const fleets = itemState.robots;
    const fleetKeys = Object.keys(fleets);

    fleetKeys.forEach((fleet) => {
      fleets[fleet].robots.forEach((robot) => {
        switch (robot.mode.mode) {
          case RomiCore.RobotMode.MODE_ADAPTER_ERROR:
          case RomiCore.RobotMode.MODE_EMERGENCY:
            spoiltEquipment.push(robot.name + ` - ${robotModeToString(robot.mode)}`);
            modeCounter.outOfOrder += 1;
            break;
          case RomiCore.RobotMode.MODE_CHARGING:
            modeCounter.operational += 1;
            modeCounter.charging += 1;
            break;
          case RomiCore.RobotMode.MODE_DOCKING:
          case RomiCore.RobotMode.MODE_GOING_HOME:
          case RomiCore.RobotMode.MODE_MOVING:
          case RomiCore.RobotMode.MODE_PAUSED:
          case RomiCore.RobotMode.MODE_WAITING:
            modeCounter.operational += 1;
            break;
          case RomiCore.RobotMode.MODE_IDLE:
            modeCounter.operational += 1;
            modeCounter.idle += 1;
            break;
        }
      });
    });
    return {
      item: 'Robots',
      summary: {
        operational: modeCounter.operational,
        outOfOrder: modeCounter.outOfOrder,
        charging: modeCounter.charging,
        idle: modeCounter.idle,
      },
      outOfOrder: spoiltEquipment,
    };
  };

  const getAllEquipmentSummary = () => {
    const getDoor = getDoorSummary();
    const getLift = getLiftSummary();
    const getDispensers = getDispenserSummary();
    const getRobots = getRobotSummary();
    return {
      door: getDoor,
      lift: getLift,
      dispenser: getDispensers,
      robot: getRobots,
    };
  };

  const toggleBannerColor = (): boolean => {
    const equipment = getAllEquipmentSummary();
    return (
      equipment.door.summary.outOfOrder +
        equipment.lift.summary.outOfOrder +
        equipment.dispenser.summary.outOfOrder +
        equipment.robot.summary.outOfOrder !==
      0
    );
  };

  const getSpoiltEquipment = (): SpoiltItem[] => {
    const equipment = getAllEquipmentSummary();
    const itemHolder = [
      ...equipment.door.outOfOrder,
      ...equipment.lift.outOfOrder,
      ...equipment.dispenser.outOfOrder,
      ...equipment.robot.outOfOrder,
    ];
    return [...itemHolder.map((item) => ({ itemNameAndState: item }))];
  };

  return (
    <React.Fragment>
      <SystemSummaryBanner bannerUrl={'/favicon.ico'} isError={toggleBannerColor()} />
      <div className={classes.root}>
        <SystemSummaryAlert notifications={notifications} />
        <Divider className={classes.divider} />

        {getSpoiltEquipment().length > 0 ? (
          <React.Fragment>
            <SystemSummarySpoiltItems spoiltItems={getSpoiltEquipment()} />
            <Divider className={classes.divider} />
          </React.Fragment>
        ) : null}

        <Typography variant="h5" className={classes.systemSummaryHeader}>
          Systems Summary
        </Typography>

        <SystemSummaryItemState itemSummary={getDoorSummary()} onClick={handleMainMenuDoorsClick} />
        <SystemSummaryItemState itemSummary={getLiftSummary()} onClick={handleMainMenuLiftsClick} />
        <SystemSummaryItemState
          itemSummary={getDispenserSummary()}
          onClick={handleMainMenuDispensersClick}
        />
        <SystemSummaryItemState
          itemSummary={getRobotSummary()}
          onClick={handleMainMenuRobotsClick}
        />
        <Divider className={classes.divider} />

        <Typography variant="h6">Task Statuses</Typography>
        <SystemSummaryTaskState tasks={tasks} />
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
