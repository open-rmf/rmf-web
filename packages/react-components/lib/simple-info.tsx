import Divider from '@material-ui/core/Divider';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import { makeStyles } from '@material-ui/core/styles';
import Typography from '@material-ui/core/Typography';
import React from 'react';

export interface SimpleInfo {
  data: Record<string, number | string | number[] | string[]>;
}

const useStyles = makeStyles((theme) => ({
  container: {
    display: 'flex',
    flexFlow: 'column',
  },
  item: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
}));

export const SimpleInfo = (props: SimpleInfo) => {
  const { data } = props;
  const classes = useStyles();

  const renderPrimitive = (name: string, value: string | number) => (
    <div className={classes.item}>
      <Typography variant="body1">{`${name}:`}</Typography>
      <Typography variant="body1">{value}</Typography>
    </div>
  );

  const renderArray = (name: string, values: any[]) => (
    <div className={classes.item}>
      <Typography variant="body1">{`${name}:`}</Typography>
      <List dense>
        {values.map((value, i) => (
          <ListItem key={i}>
            <Typography variant="body1">{value}</Typography>
          </ListItem>
        ))}
      </List>
    </div>
  );

  const renderLine = (name: string, value: any) => {
    switch (typeof value) {
      case 'object':
        if (Array.isArray(value)) {
          return renderArray(name, value);
        } else {
          throw Error('nested object is not supported');
        }
        break;
      case 'function':
      case 'symbol':
        break;
      default:
        return renderPrimitive(name, value);
    }
  };

  return (
    <div className={classes.container}>
      {Object.entries(data).map(([key, value]) => (
        <>
          {renderLine(key, value)}
          <Divider />
        </>
      ))}
    </div>
  );
};

export default SimpleInfo;
