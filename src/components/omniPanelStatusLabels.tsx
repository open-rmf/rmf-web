import React from 'react';
import { Typography } from '@material-ui/core';

interface LabelProps {
  hideTextStyle: string;
  modalLabelClass: string;
  modeText?: string;
  name?: string;
}

export default function OmniPanelStatusLabels(props: LabelProps) {

  const { hideTextStyle, modalLabelClass, modeText, name } = props;

  return (
    <React.Fragment>
      <Typography variant="h6" className={hideTextStyle}>
        {name}
      </Typography>
        <Typography className={modalLabelClass} variant='button'>
          {modeText}
      </Typography>
    </React.Fragment>
  )
}
