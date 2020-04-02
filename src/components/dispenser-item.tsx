import {
  Divider,
  ExpansionPanelDetails,
  ExpansionPanelProps,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  useTheme,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export interface DispenserItemProps extends ExpansionPanelProps {
  dispenserState?: Readonly<RomiCore.DispenserState>;
}

export const DispenserItem = React.forwardRef(function(
    props: DispenserItemProps,
    ref: React.Ref<HTMLElement>,
): React.ReactElement {

});

export default DispenserItem;
