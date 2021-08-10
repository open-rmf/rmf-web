import React from 'react';
import { Paper, IconButton, makeStyles, Grid, Typography, Box } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import FilterListIcon from '@material-ui/icons/FilterList';
import ArrowDownwardIcon from '@material-ui/icons/ArrowDownward';
import ArrowUpwardIcon from '@material-ui/icons/ArrowUpward';

import { LiftTable } from './lift-table';
import { LiftPanelProps, LiftCellProps } from './lift-utils';
import { motionStateToString, doorStateToString } from './lift-utils';

const useStyles = makeStyles((theme) => ({
  buttonBar: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: '0px',
    backgroundColor: theme.palette.primary.main,
  },
  grid: {
    padding: '1rem',
  },
  cellPaper: {
    padding: '0.5rem',
    backgroundColor: theme.palette.info.light,
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
  buttonGroup: {
    display: 'flex',
    justifyContent: 'center',
  },
  iconMoving: {
    color: theme.palette.success.dark,
  },
  iconOtherStates: {
    color: 'white',
  },
  doorLabelOpen: {
    backgroundColor: theme.palette.success.main,
  },
  doorLabelClosed: {
    backgroundColor: theme.palette.error.main,
  },
  doorLabelMoving: {
    backgroundColor: theme.palette.warning.main,
  },
}));

const LiftCell = (props: LiftCellProps): JSX.Element => {
  const { lift, liftState } = props;
  const classes = useStyles();

  const currMotion = motionStateToString(liftState.motion_state);
  const getMotionArrowColor = (currMotion: string, arrowDirection: string) => {
    return currMotion === arrowDirection ? classes.iconMoving : classes.iconOtherStates;
  };

  const currDoorMotion = doorStateToString(liftState.door_state);
  const getDoorMotionColor = (doorMotion: string) => {
    switch (doorMotion) {
      case 'Open':
        return `${classes.doorLabelOpen}`;
      case 'Closed':
        return `${classes.doorLabelClosed}`;
      case 'Moving':
        return `${classes.doorLabelMoving}`;
      default:
        return '';
    }
  };

  return (
    <Paper className={classes.cellPaper}>
      <Grid container direction="row">
        <Grid item xs={9}>
          <Typography align="center">{lift.name}</Typography>
          <Box border={1} borderColor="divider" m={0.5}>
            <Typography align="center">{liftState.destination_floor}</Typography>
          </Box>
          <Typography align="center" className={getDoorMotionColor(currDoorMotion)}>
            {currDoorMotion}
          </Typography>
        </Grid>
        <Grid item xs>
          <Typography align="center" className={getMotionArrowColor(currMotion, 'Up')}>
            <ArrowUpwardIcon />
          </Typography>
          <Typography align="center">{liftState.current_floor}</Typography>
          <Typography align="center" className={getMotionArrowColor(currMotion, 'Down')}>
            <ArrowDownwardIcon />
          </Typography>
        </Grid>
      </Grid>
    </Paper>
  );
};

export function LiftPanel(props: LiftPanelProps) {
  const { lifts, liftStates } = props;
  const classes = useStyles();

  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <div>
      <Paper className={classes.buttonBar}>
        <IconButton className={classes.itemIcon}>
          <FilterListIcon />
        </IconButton>
        <IconButton
          aria-label="view-mode"
          className={classes.itemIcon}
          onClick={() => setIsCellView(!isCellView)}
        >
          {isCellView ? <ViewListIcon /> : <ViewModuleIcon />}
        </IconButton>
      </Paper>
      <Grid className={classes.grid} container direction="row" spacing={1}>
        {isCellView ? (
          lifts.map((lift, i) => {
            return (
              <Grid item xs={4} key={`${lift.name}_${i}`}>
                <LiftCell lift={lift} liftState={liftStates[lift.name]} />
              </Grid>
            );
          })
        ) : (
          <LiftTable lifts={lifts} liftStates={liftStates} />
        )}
      </Grid>
    </div>
  );
}
