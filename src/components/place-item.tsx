import {
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelProps,
  ExpansionPanelSummary,
  ListItemText,
  makeStyles,
  Typography,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

const useStyles = makeStyles(theme => ({
  summary: {
    marginTop: 0,
    marginBottom: 0,
  },

  details: {
    flexFlow: 'column',
  },

  detailLine: {
    width: '100%',
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
}));

export interface PlaceItemProps extends Omit<ExpansionPanelProps, 'children'> {
  place: RomiCore.Place;
  level: RomiCore.Level;
}

export const PlaceItem = React.forwardRef(function(
  props: PlaceItemProps,
  ref: React.Ref<HTMLElement>,
): React.ReactElement {
  const { place, level, ...otherProps } = props;
  const classes = useStyles();

  return (
    <ExpansionPanel ref={ref} {...otherProps}>
      <ExpansionPanelSummary expandIcon={<ExpandMoreIcon />}>
        <ListItemText
          className={classes.summary}
          primary={place.name}
          primaryTypographyProps={{ variant: 'h5' }}
          secondary={level.name}
        />
      </ExpansionPanelSummary>
      <ExpansionPanelDetails className={classes.details}>
        <div className={classes.detailLine}>
          <Typography>Location:</Typography>
          <Typography>{`${level.name} (${place.x.toFixed(3)}, ${place.y.toFixed(
            3,
          )}, ${place.yaw.toFixed(3)})`}</Typography>
        </div>
        <Divider />
        <div className={classes.detailLine}>
          <Typography>Position Tolerance:</Typography>
          <Typography>{place.position_tolerance.toFixed(3)}</Typography>
        </div>
        <Divider />
        <div className={classes.detailLine}>
          <Typography>Yaw Tolerance:</Typography>
          <Typography>{place.yaw_tolerance.toFixed(3)}</Typography>
        </div>
      </ExpansionPanelDetails>
    </ExpansionPanel>
  );
});
