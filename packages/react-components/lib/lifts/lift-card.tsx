import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import {
  Box,
  Card,
  CardContent,
  CardProps,
  Grid,
  SxProps,
  Typography,
  useTheme,
} from '@mui/material';
import React from 'react';
import { LiftState } from 'rmf-models';
import { doorStateToString, motionStateToString } from './lift-utils';

export interface LiftCardProps extends CardProps {
  name: string;
  motionState?: number;
  doorState?: number;
  currentFloor?: string;
  destinationFloor?: string;
}

export function LiftCard({
  name,
  motionState,
  doorState,
  currentFloor,
  destinationFloor,
  children,
  ...cardProps
}: LiftCardProps): JSX.Element {
  const theme = useTheme();
  const currMotion = motionStateToString(motionState);
  const motionArrowActiveStyle: SxProps = {
    color: theme.palette.primary.main,
  };
  const motionArrowIdleStyle: SxProps = {
    color: theme.palette.action.disabled,
    opacity: theme.palette.action.disabledOpacity,
  };
  const currDoorMotion = doorStateToString(doorState);

  const doorStateLabelStyle: SxProps = (() => {
    switch (doorState) {
      case LiftState.DOOR_OPEN:
        return {
          backgroundColor: theme.palette.success.main,
          color: theme.palette.success.contrastText,
        };
      case LiftState.DOOR_CLOSED:
        return {
          backgroundColor: theme.palette.error.main,
          color: theme.palette.error.contrastText,
        };
      case LiftState.DOOR_MOVING:
        return {
          backgroundColor: theme.palette.warning.main,
          color: theme.palette.warning.contrastText,
        };
      default:
        return {
          backgroundColor: theme.palette.action.disabledBackground,
          color: theme.palette.action.disabled,
        };
    }
  })();

  return (
    <Card variant="outlined" {...cardProps}>
      <CardContent>
        <Grid container>
          <Grid item xs={9}>
            <Typography noWrap align="center" variant="h6" title={name}>
              {name}
            </Typography>
            <Box component="div" border={1} borderColor="divider">
              <Typography align="center">{destinationFloor || 'Unknown'}</Typography>
            </Box>
            <Typography variant="body2" align="center" sx={doorStateLabelStyle}>
              {currDoorMotion}
            </Typography>
          </Grid>
          <Grid item xs sx={{ marginLeft: theme.spacing(2) }}>
            <Typography
              align="center"
              sx={currMotion === 'Up' ? motionArrowActiveStyle : motionArrowIdleStyle}
            >
              <ArrowUpwardIcon />
            </Typography>
            <Typography align="center">{currentFloor || '?'}</Typography>
            <Typography
              align="center"
              sx={currMotion === 'Down' ? motionArrowActiveStyle : motionArrowIdleStyle}
            >
              <ArrowDownwardIcon />
            </Typography>
          </Grid>
        </Grid>
      </CardContent>
      {children}
    </Card>
  );
}
