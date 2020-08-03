import React from 'react';
import { Typography } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

interface LabelProps {
  hideText: string;
  modalLabelClass: string;
  modeText?: string;
  liftState?: Readonly<RomiCore.LiftState>;
  liftName?: string;
  name?: string;
}

export default function Labels(props: LabelProps) {

  const { hideText, modalLabelClass, modeText, liftState, name, liftName } = props;

  return (
    <React.Fragment>
      <Typography variant="h6" className={hideText}>
        {liftName ? liftName : name}
      </Typography>
      {
        liftName ? 
        <Typography className={modalLabelClass} variant="button">
          {liftState ? liftState.current_floor : 'N/A'}
        </Typography>
        :
        <Typography className={modalLabelClass} variant='button'>
          {modeText}
      </Typography>
      }
    </React.Fragment>
  )
}