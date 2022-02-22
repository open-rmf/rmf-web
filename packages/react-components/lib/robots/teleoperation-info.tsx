import { Divider, Grid, styled, useTheme } from '@mui/material';
import { TeleoperationApi } from 'api-client';
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
  teleoperationApi: TeleoperationApi;
}

export function TeleoperationInfo({
  robotName,
  teleoperationApi,
}: TeleoperationInfoProps): JSX.Element {
  const theme = useTheme();
  // TODO(BH): Make this globally configurable
  const name = robotName.trim();
  const url = 'https://meet.jit.si/' + name;

  return (
    <StyledDiv style={{ height: '100%', width: '100%' }}>
      <Divider />
      <Grid container style={{ height: '100%', width: '100%' }}>
        <Grid container item xs={12} justifyContent="center">
          <iframe
            allow="camera; microphone; fullscreen; display-capture; autoplay"
            src={url}
            style={{ height: '95%', width: '100%', border: '0px' }}
          ></iframe>
          <button
            onClick={() =>
              teleoperationApi.joinVideoRoomTeleoperationNameVideoJoinPost(name, {
                target_name: name,
                url: url,
              })
            }
          >
            Join
          </button>
          <button onClick={() => console.log('LEAVE')}>Leave</button>
          <Divider />
          <button onClick={() => console.log('LEFT')}>Turn Left</button>
          <button onClick={() => console.log('DRIVE')}>Drive</button>
          <button onClick={() => console.log('REVERSE')}>Reverse</button>
          <button onClick={() => console.log('RIGHT')}>Turn Right</button>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
