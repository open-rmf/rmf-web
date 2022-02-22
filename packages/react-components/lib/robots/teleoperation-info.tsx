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

          <div>
            <button
              onClick={() =>
                teleoperationApi.joinVideoRoomTeleoperationNameVideoJoinPost(name, {
                  url: url,
                })
              }
            >
              Join
            </button>
            <button
              onClick={() =>
                teleoperationApi.leaveVideoRoomTeleoperationNameVideoLeavePost(name, {})
              }
            >
              Leave
            </button>
            <button
              onClick={() =>
                teleoperationApi.moveTeleoperationNameMovePost(name, {
                  name: 'camera',
                  x_m: 0.0,
                  y_m: 0.0,
                  z_m: 0.0,
                  roll_deg: 0.0,
                  pitch_deg: 1.0,
                  yaw_deg: 0.0,
                })
              }
            >
              Camera Up
            </button>
            <button
              onClick={() =>
                teleoperationApi.moveTeleoperationNameMovePost(name, {
                  name: 'camera',
                  x_m: 0.0,
                  y_m: 0.0,
                  z_m: 0.0,
                  roll_deg: 0.0,
                  pitch_deg: -1.0,
                  yaw_deg: 0.0,
                })
              }
            >
              Camera Down
            </button>
          </div>

          <div>
            <button
              onClick={() =>
                teleoperationApi.moveTeleoperationNameMovePost(name, {
                  name: name,
                  x_m: 0.0,
                  y_m: 0.0,
                  z_m: 0.0,
                  roll_deg: 0.0,
                  pitch_deg: 0.0,
                  yaw_deg: -0.5,
                })
              }
            >
              Turn Left
            </button>
            <button
              onClick={() =>
                teleoperationApi.moveTeleoperationNameMovePost(name, {
                  name: name,
                  x_m: -1.0,
                  y_m: 0.0,
                  z_m: 0.0,
                  roll_deg: 0.0,
                  pitch_deg: 0.0,
                  yaw_deg: 0.0,
                })
              }
            >
              Down
            </button>
            <button
              onClick={() =>
                teleoperationApi.moveTeleoperationNameMovePost(name, {
                  name: name,
                  x_m: 1.0,
                  y_m: 0.0,
                  z_m: 0.0,
                  roll_deg: 0.0,
                  pitch_deg: 0.0,
                  yaw_deg: 0.0,
                })
              }
            >
              Up
            </button>
            <button
              onClick={() =>
                teleoperationApi.moveTeleoperationNameMovePost(name, {
                  name: name,
                  x_m: 0.0,
                  y_m: 0.0,
                  z_m: 0.0,
                  roll_deg: 0.0,
                  pitch_deg: 0.0,
                  yaw_deg: 0.5,
                })
              }
            >
              Turn Right
            </button>
          </div>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
