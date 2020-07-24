import { Divider, Typography, makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

interface robotInformationProps {
  robot: Readonly<RomiCore.RobotState>;
}

export const RobotInformation = (props: robotInformationProps) => {
  const { robot } = props;
  const classes = useStyles();
  return (
    <>
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Name:</Typography>
        <Typography variant="body1">{robot.name}</Typography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Model:</Typography>
        <Typography variant="body1">{robot.model}</Typography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Level:</Typography>
        <Typography variant="body1">{robot.location.level_name}</Typography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Position:</Typography>
        <Typography data-role="position" variant="body1">
          {`(${robot.location.x.toFixed(3)}, ${robot.location.y.toFixed(3)})`}
        </Typography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Yaw:</Typography>
        <Typography variant="body1">{robot.location.yaw.toFixed(3)}</Typography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Task Id:</Typography>
        <Typography variant="body1" noWrap>
          {robot.task_id}
        </Typography>
      </div>
      <Divider />
      <div className={classes.expansionDetailLine}>
        <Typography variant="body1">Battery:</Typography>
        <Typography variant="body1">{robot.battery_percent}</Typography>
      </div>
    </>
  );
};

const useStyles = makeStyles(theme => ({
  expansionDetailLine: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
}));
