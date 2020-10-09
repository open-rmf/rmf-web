import { makeStyles, Typography } from '@material-ui/core';
import React from 'react';

export interface StatusLabelProps {
  modalLabelClass: string;
  modeText?: string;
  name?: string;
  hideTextStyleOverride?: string;
}

const useStyles = makeStyles({
  hideText: {
    overflow: 'hidden',
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
    maxWidth: '10rem',
  },
});

export default function StatusLabel(props: StatusLabelProps) {
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
