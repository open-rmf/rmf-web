import React from 'react';
import { Typography, makeStyles } from '@material-ui/core';

export interface LabelProps {
  modalLabelClass: string;
  modeText?: string;
  name?: string;
  hideTextStyleOverride?: string;
}

export const useStyles = makeStyles(theme => ({
  hideText: {
    overflow: 'hidden',
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
    maxWidth: '10rem',
  },
}));

export default function OmniPanelStatusLabels(props: LabelProps) {
  const classes = useStyles();
  const { modalLabelClass, modeText, name, hideTextStyleOverride } = props;

  return (
    <React.Fragment>
      <Typography
        variant="h6"
        className={hideTextStyleOverride ? hideTextStyleOverride : classes.hideText}
      >
        {name}
      </Typography>
      <Typography className={modalLabelClass} variant="button">
        {modeText}
      </Typography>
    </React.Fragment>
  );
}
