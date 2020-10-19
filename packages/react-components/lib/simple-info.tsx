import Divider from '@material-ui/core/Divider';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import { makeStyles } from '@material-ui/core/styles';
import Typography from '@material-ui/core/Typography';
import React from 'react';

type DataValueTypePrimitive = number | string;
type DataValueTypeArray = DataValueTypePrimitive[];
type DataValueType = DataValueTypePrimitive | DataValueTypeArray;

export interface SimpleInfoData<T extends DataValueType = DataValueType> {
  name: string;
  value: T;
  className?: T extends DataValueTypeArray ? string | string[] : string;
}

export interface SimpleInfo {
  data: SimpleInfoData[];
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

export const SimpleInfo = (props: SimpleInfo): JSX.Element => {
  const { data } = props;
  const classes = useStyles();

  const renderPrimitive = ({ name, value, className }: SimpleInfoData<DataValueTypePrimitive>) => (
    <div className={classes.item}>
      <Typography variant="body1">{`${name}:`}</Typography>
      <Typography variant="body1" className={className}>
        {value}
      </Typography>
    </div>
  );

  const renderArray = ({ name, value, className }: SimpleInfoData<DataValueTypeArray>) => (
    <div className={classes.item}>
      <Typography variant="body1">{`${name}:`}</Typography>
      <List dense>
        {value.map((item, i) => (
          <ListItem key={i}>
            <Typography
              variant="body1"
              className={Array.isArray(className) ? className[i] : className}
            >
              {item}
            </Typography>
          </ListItem>
        ))}
      </List>
    </div>
  );

  const renderLine = (data: SimpleInfoData) => {
    switch (typeof data.value) {
      case 'object':
        if (Array.isArray(data.value)) {
          return renderArray(data as SimpleInfoData<DataValueTypeArray>);
        } else {
          throw Error('nested object is not supported');
        }
        break;
      case 'function':
      case 'symbol':
        break;
      default:
        return renderPrimitive(data as SimpleInfoData<DataValueTypePrimitive>);
    }
  };

  return (
    <div className={classes.container}>
      {data.map((item) => (
        <React.Fragment key={item.name}>
          {renderLine(item)}
          <Divider />
        </React.Fragment>
      ))}
    </div>
  );
};

export default SimpleInfo;
