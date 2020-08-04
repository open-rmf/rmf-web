import React from 'react';
import { Typography, makeStyles } from '@material-ui/core';

interface LabelProps {
  modalLabelClass: string;
  modeText?: string;
  name?: string;
}

const useStyles = makeStyles(theme => ({
  hideText: {
    overflow: "hidden",
    textOverflow: "ellipsis",
    whiteSpace: "nowrap",
    maxWidth: "10rem",
  }
}));

export default function OmniPanelStatusLabels(props: LabelProps) {
  const classes = useStyles();
  const { modalLabelClass, modeText, name } = props;

  return (
    <React.Fragment>
      <Typography variant="h6" className={classes.hideText}>
        {name}
      </Typography>
      <Typography className={modalLabelClass} variant='button'>
        {modeText}
      </Typography>
    </React.Fragment>
  )
}
