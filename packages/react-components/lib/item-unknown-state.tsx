import { makeStyles, Divider } from '@material-ui/core';
import Typography from '@material-ui/core/Typography';
import React from 'react';

const useStyles = makeStyles(() => ({
  typography: {
    padding: '0.5rem 0',
  },
}));

interface ItemUnknownProps {
  unknownItem: string;
}

export default function ItemUnknownState(props: ItemUnknownProps): JSX.Element {
  const { unknownItem } = props;
  const classes = useStyles();

  return (
    <React.Fragment>
      <Divider />
      <Typography className={classes.typography} align="center" variant="body1">
        State of {unknownItem} is unknown
      </Typography>
    </React.Fragment>
  );
}
