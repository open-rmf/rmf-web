import { Divider, Grid, styled, useTheme, Typography } from '@mui/material';
import { SioClient } from 'api-client';
import React from 'react';

const classes = {
  button: 'robot-info-button',
};
const StyledDiv = styled('div')(() => ({
  [`& .${classes.button}`]: {
    '&:hover': {
      background: 'none',
      cursor: 'default',
    },
  },
}));

export interface TeleoperationInfoProps {
  robotName: string;
  sioClient: SioClient;
}

export function TeleoperationInfo({ robotName }: TeleoperationInfoProps): JSX.Element {
  const theme = useTheme();

  return (
    <StyledDiv style={{ height: '100%', width: '100%' }}>
      <Divider />
      <Grid container style={{ height: '100%', width: '100%' }}>
        <Grid container item xs={12} justifyContent="center">
          <iframe
            allow="camera; microphone; fullscreen; display-capture; autoplay"
            src={'https://meet.jit.si/' + robotName.trim()}
            style={{ height: '95%', width: '100%', border: '0px' }}
          ></iframe>
          <button>Join</button>
          <button>Leave</button>
          <Divider />
          <button>Turn Left</button>
          <button>Drive</button>
          <button>Reverse</button>
          <button>Turn Right</button>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
