import {
  ExpansionPanelSummary,
  ListItemText,
  makeStyles,
  ExpansionPanelDetails,
  Typography,
  Divider,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import SpotlightExpansionPanel, { SpotlightValue } from './spotlight-expansion-panel';

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

export interface PlacesPanelProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
  spotlight?: Readonly<SpotlightValue<string>>;
  onPlaceClick?(place: RomiCore.Place): void;
}

export default function PlacesPanel(props: PlacesPanelProps): React.ReactElement {
  const classes = useStyles();
  const spotlightRef = React.useRef<HTMLElement>(null);

  React.useEffect(() => {
    spotlightRef.current?.scrollIntoView();
  }, [props.spotlight]);

  return (
    <React.Fragment>
      {props.buildingMap.levels.flatMap(level =>
        level.places.map(place => (
          <SpotlightExpansionPanel key={place.name} index={place.name} spotlight={props.spotlight}>
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
          </SpotlightExpansionPanel>
        )),
      )}
    </React.Fragment>
  );
}
