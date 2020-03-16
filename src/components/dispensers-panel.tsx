import {
  Button,
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  useTheme,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import SpotlightExpansionPanel, { SpotlightValue } from './spotlight-expansion-panel';

import React from 'react';

function dispenserModeToString(dispenserState?: RomiCore.DispenserState): string {
  if (!dispenserState) {
    return 'UNKNOWN';
  }
  switch (dispenserState.mode) {
    case RomiCore.DispenserState.IDLE:
      return 'IDLE';
    case RomiCore.DispenserState.BUSY:
      return 'BUSY';
    case RomiCore.DispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'UNKNOWN';
  }
}

const useStyles = makeStyles(theme => ({
  expansionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  expansionDetail: {
    flexFlow: 'column',
  },

  expansionDetialLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
}));

export interface DispenserPanelProps {
  transport?: Readonly<RomiCore.Transport>;
  dispenserStates: Readonly<Record<string, RomiCore.DispenserState | undefined>>;
  spotlight?: SpotlightValue<string>;
}

export default function DispensersPanel(props: DispenserPanelProps): React.ReactElement {
  const theme = useTheme();
  const classes = useStyles();

  const dispensers = Object.keys(props.dispenserStates).map( (guid, index) => {
    const state = props.dispenserStates[guid];
    return (
        <SpotlightExpansionPanel 
            key={guid} index={guid}
            spotlight={props.spotlight} 
            TransitionProps={{ unmountOnExit: true }}>
          <ExpansionPanelSummary
            classes={{ content: classes.expansionSummaryContent }}
            expandIcon={<ExpandMoreIcon />}
          >
            <Typography variant="h5">{guid}</Typography>
          </ExpansionPanelSummary>
        </SpotlightExpansionPanel>
    );
  })

  return <React.Fragment>{dispensers}</React.Fragment>;
  // return <h1>This is the dispensers panel</h1>;
}
