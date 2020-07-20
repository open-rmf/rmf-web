import {
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelProps,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
} from '@material-ui/core';
import { AntTabs, AntTab, TabPanel } from './tab';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import { RobotInformation } from './robot-item-information';
import { RobotLoopForm } from './robot-item-form';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import fakePlaces from '../mock/data/places';
import React from 'react';

export interface RobotItemProps extends Omit<ExpansionPanelProps, 'children'> {
  fleetName: string;
  robot: Readonly<RomiCore.RobotState>;
  onRobotClick?(robot: RomiCore.RobotState): void;
  requestLoop(
    fleetName: string,
    numLoops: number,
    startLocationPoint: string,
    endLocationPoint: string,
  ): void;
}

export const RobotItem = React.forwardRef(function(
  props: RobotItemProps,
  ref: React.Ref<HTMLElement>,
): React.ReactElement {
  const { robot, onRobotClick, requestLoop, fleetName, ...otherProps } = props;
  const [value, setValue] = React.useState(0);
  const handleChange = (event: React.ChangeEvent<{}>, newValue: number) => {
    setValue(newValue);
  };
  const classes = useStyles();
  const listOfPlaces = fakePlaces()[fleetName];
  return (
    <ExpansionPanel ref={ref} data-component="RobotItem" data-name={robot.name} {...otherProps}>
      <ExpansionPanelSummary
        classes={{ content: classes.expansionSummaryContent }}
        expandIcon={<ExpandMoreIcon />}
      >
        <Typography variant="h5" className={classes.hideText}>{robot.name}</Typography>
        <Typography className={classes.robotStatusLabel} variant="button">
          {robotModeToString(robot.mode)}
        </Typography>
      </ExpansionPanelSummary>
      <ExpansionPanelDetails data-role="details" className={classes.expansionDetail}>
        <AntTabs centered value={value} onChange={handleChange} aria-label="ant example">
          <AntTab label="Info" />
          <AntTab label="Loop" />
          <AntTab label="Delivery" />
        </AntTabs>
        <TabPanel value={value} index={0}>
          <RobotInformation robot={robot} />
        </TabPanel>
        <TabPanel value={value} index={1}>
          {!!listOfPlaces && (
            <RobotLoopForm
              requestLoop={requestLoop}
              fleetName={fleetName}
              listOfPlaces={listOfPlaces}
            />
          )}
        </TabPanel>
      </ExpansionPanelDetails>
    </ExpansionPanel>
  );
});

export default RobotItem;

const useStyles = makeStyles(theme => ({
  expansionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  expansionDetail: {
    flexFlow: 'column',
    overflowX: 'auto',
    padding: '0.8rem'
  },

  expansionDetailLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },

  robotStatusLabel: {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    borderColor: theme.palette.info.main,
    border: 2,
    padding: 5,
    minWidth: '4rem',
    textAlign: 'center',
  },

  hideText: {
    overflow: "hidden",
    textOverflow: "ellipsis",
    whiteSpace: "nowrap",
    maxWidth: "10rem"
  }
}));

function robotModeToString(robotMode: RomiCore.RobotMode): string {
  switch (robotMode.mode) {
    case RomiCore.RobotMode.MODE_CHARGING:
      return 'Charging';
    case RomiCore.RobotMode.MODE_DOCKING:
      return 'Docking';
    case RomiCore.RobotMode.MODE_EMERGENCY:
      return 'Emergency';
    case RomiCore.RobotMode.MODE_GOING_HOME:
      return 'Going Home';
    case RomiCore.RobotMode.MODE_IDLE:
      return 'Idle';
    case RomiCore.RobotMode.MODE_MOVING:
      return 'Moving';
    case RomiCore.RobotMode.MODE_PAUSED:
      return 'Paused';
    case RomiCore.RobotMode.MODE_WAITING:
      return 'Waiting';
    default:
      return `Unknown (${robotMode.mode})`;
  }
}
