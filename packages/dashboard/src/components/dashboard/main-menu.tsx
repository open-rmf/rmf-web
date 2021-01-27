import { Divider, makeStyles, Typography, Button } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import DashboardTooltip from 'react-components/lib/tooltip';
import { TooltipsContext } from '../app-contexts';
import { OmniPanelViewIndex } from './dashboard';
import { MainMenuItemState } from './main-menu-item-state';

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
    margin: '1rem 0',
  },
}));

export interface ItemState {
  doors: Record<string, RomiCore.DoorState>;
  lifts: Record<string, RomiCore.LiftState>;
  robots: Record<string, RomiCore.FleetState>;
  dispensers: Record<string, RomiCore.DispenserState>;
}

export interface ItemSummaryState {
  [key: string]: number;
}

export interface ItemSummary {
  item: string;
  summary: ItemSummaryState[];
}

export interface MainMenuProps {
  pushView(view: OmniPanelViewIndex): void;
  itemState: ItemState;
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  const { showTooltips } = React.useContext(TooltipsContext);
  const { pushView, itemState } = props;
  debug('render');
  const classes = useStyles();
  console.log(itemState);
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
    let modeCounter = { operational: 0, broken: 0 };
    const doors = itemState.doors;
    const doorKeys = Object.keys(itemState.doors);

    doorKeys.forEach((door) => {
      switch (doors[door].current_mode.value) {
        case 0:
        case 1:
        case 2:
          modeCounter['operational'] += 1;
          break;
        default:
          modeCounter['broken'] += 1;
          break;
      }
    });
    return {
      item: 'Door',
      summary: [{ operational: modeCounter.operational }, { broken: modeCounter.broken }],
    };
  };

  const getLiftSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, broken: 0 };
    const lifts = itemState.lifts;
    const liftKeys = Object.keys(itemState.lifts);

    liftKeys.forEach((lift) => {
      switch (lifts[lift].current_mode) {
        case 1:
        case 2:
        case 4:
          modeCounter['operational'] += 1;
          break;
        case 3:
        case 5:
        case 0:
          modeCounter['broken'] += 1;
          break;
      }
    });
    return {
      item: 'Lift',
      summary: [{ operational: modeCounter.operational }, { broken: modeCounter.broken }],
    };
  };

  const getDispenserSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, broken: 0 };
    const dispensers = itemState.dispensers;
    const dispenserKeys = Object.keys(itemState.dispensers);

    dispenserKeys.forEach((dispenser) => {
      switch (dispensers[dispenser].mode) {
        case 0:
        case 1:
        case 2:
          modeCounter['operational'] += 1;
          break;
        default:
          modeCounter['broken'] += 1;
          break;
      }
    });

    return {
      item: 'Dispensers',
      summary: [{ operational: modeCounter.operational }, { broken: modeCounter.broken }],
    };
  };

  return (
    <div className={classes.root}>
      <Typography variant="h5">Systems Summary</Typography>
      <MainMenuItemState itemSummary={getDoorSummary()} />
      <MainMenuItemState itemSummary={getLiftSummary()} />
      <MainMenuItemState itemSummary={getDispenserSummary()} />
      <Divider className={classes.divider} />

      <Typography variant="h6">Item Details</Typography>
      <Button
        color="primary"
        className={classes.buttons}
        variant="contained"
        onClick={handleMainMenuDoorsClick}
      >
        <Typography variant="body1">Doors</Typography>
      </Button>
      <Button
        color="primary"
        className={classes.buttons}
        variant="contained"
        onClick={handleMainMenuLiftsClick}
      >
        <Typography variant="body1">Lifts</Typography>
      </Button>
      <Button
        color="primary"
        className={classes.buttons}
        variant="contained"
        onClick={handleMainMenuRobotsClick}
      >
        <Typography variant="body1">Robots</Typography>
      </Button>
      <Button
        color="primary"
        className={classes.buttons}
        variant="contained"
        onClick={handleMainMenuDispensersClick}
      >
        <Typography variant="body1">Dispensers</Typography>
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
      <Divider className={classes.divider} />

      <Typography variant="h6">Task Statuses</Typography>
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
    </div>
  );
});

export default MainMenu;
