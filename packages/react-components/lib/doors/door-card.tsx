import { Card, CardContent, CardProps, Grid, SxProps, Typography, useTheme } from '@mui/material';
import React from 'react';
import { DoorMode } from 'rmf-models';
import { doorModeToString, doorTypeToString } from './door-utils';

export interface DoorCardProps extends CardProps {
  name: string;
  level: string;
  mode: number;
  type: number;
}

export function DoorCard({
  name,
  level,
  mode,
  type,
  children,
  ...cardProps
}: DoorCardProps): JSX.Element {
  const theme = useTheme();
  const labelStyle = React.useMemo<SxProps>(() => {
    switch (mode) {
      case DoorMode.MODE_OPEN:
        return {
          backgroundColor: theme.palette.success.main,
          color: theme.palette.success.contrastText,
        };
      case DoorMode.MODE_CLOSED:
        return {
          backgroundColor: theme.palette.error.main,
          color: theme.palette.error.contrastText,
        };
      case DoorMode.MODE_MOVING:
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
  }, [theme, mode]);

  return (
    <Card variant="outlined" {...cardProps}>
      <CardContent>
        <Typography noWrap variant="h6" align="center" title={name}>
          {name}
        </Typography>
        <Grid container direction="row" spacing={1}>
          <Grid item xs={4}>
            <Typography variant="body2" align="center">
              {level}
            </Typography>
          </Grid>
          <Grid item xs={8}>
            <Typography sx={labelStyle} variant="body2" align="center" role="status">
              {doorModeToString(mode)}
            </Typography>
          </Grid>
        </Grid>
        <Typography variant="body1" align="center" marginTop={theme.spacing(1)}>
          {doorTypeToString(type)}
        </Typography>
      </CardContent>
      {children}
    </Card>
  );
}
