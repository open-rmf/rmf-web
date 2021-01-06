import React from 'react';
import { makeStyles } from '@material-ui/core';
import { SimpleInfo, SimpleInfoData } from './simple-info';
import ErrorIcon from '@material-ui/icons/Error';
import { Typography, Grid } from '@material-ui/core';

const useStyles = makeStyles((theme) => ({
  errorIcon: {
    color: theme.palette.error.main,
  },
  errorMsg: {
    margin: '0 0.5rem',
  },
}));

interface Location {
  x: number;
  y: number;
  yaw: number;
  level_name: string;
}

type FieldsKeys = 'name' | 'value';
type DataValueTypePrimitive = number | string;
type DataValueTypeArray = DataValueTypePrimitive[];
type DataValueType = DataValueTypePrimitive | DataValueTypeArray;

export interface ItemUnknownProps {
  // fields object require both key values in FieldKeys
  fields: { [key in FieldsKeys]: DataValueType }[];
  location?: Location;
  errorMsg?: string;
}

export const ItemUnknown = (props: ItemUnknownProps): JSX.Element => {
  const classes = useStyles();
  const { fields, location, errorMsg } = props;

  if (location) {
    fields.push({
      name: 'Location',
      value: `${location.level_name} (${location.x.toFixed(3)}, ${location.y.toFixed(3)})`,
    });
  }
  const data = fields as SimpleInfoData[];
  return (
    <React.Fragment>
      <Grid container direction="row" justify="center" alignItems="center" spacing={2}>
        <Grid item>
          <Typography color="error" variant="h6" align="center">
            Error
          </Typography>
        </Grid>
        <Grid item>
          <ErrorIcon className={classes.errorIcon} />
        </Grid>
      </Grid>
      <Typography className={classes.errorMsg} color="error" variant="body1" align="center">
        {errorMsg ? errorMsg : 'Unknown error'}
      </Typography>
      <SimpleInfo infoData={data} />
    </React.Fragment>
  );
};
