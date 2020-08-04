import React from 'react';
import { Typography } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

interface LabelProps {
  hideText: string;
  modalLabelClass: string;
  modeText?: string;
  name?: string;
}

export default function Labels(props: LabelProps) {

  const { hideText, modalLabelClass, modeText, name } = props;

  return (
    <React.Fragment>
      <Typography variant="h6" className={hideText}>
        {name}
      </Typography>
        <Typography className={modalLabelClass} variant='button'>
          {modeText}
      </Typography>
    </React.Fragment>
  )
}
