import Divider from '@material-ui/core/Divider';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import { makeStyles } from '@material-ui/core/styles';
import Typography from '@material-ui/core/Typography';
import React from 'react';
import { joinClasses } from './css-utils';

type DataValueTypePrimitive = number | string;
type DataValueTypeArray = DataValueTypePrimitive[];
type DataValueType = DataValueTypePrimitive | DataValueTypeArray;

export interface SimpleInfoData<T extends DataValueType = DataValueType> {
  name: string;
  value: T;
  className?: T extends DataValueTypeArray ? string | string[] : string;
  disabled?: boolean;
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
  disabled: {
    color: theme.palette.action.disabled,
  },
}));

export const SimpleInfo = (props: SimpleInfo): JSX.Element => {
  const { data } = props;
  const classes = useStyles();

  const renderPrimitive = ({
    name,
    value,
    className,
    disabled,
  }: SimpleInfoData<DataValueTypePrimitive>) => (
    <div className={classes.item}>
      <Typography variant="body1">{`${name}:`}</Typography>
      <Typography
        variant="body1"
        className={joinClasses(disabled ? classes.disabled : undefined, className)}
      >
        {value}
      </Typography>
    </div>
  );

  const renderArray = ({
    name,
    value,
    className,
    disabled,
  }: SimpleInfoData<DataValueTypeArray>) => (
    <div className={classes.item}>
      <Typography variant="body1">{`${name}:`}</Typography>
      <List dense>
        {value.map((item, i) => (
          <ListItem key={i}>
            <Typography
              variant="body1"
              className={joinClasses(
                disabled ? classes.disabled : undefined,
                Array.isArray(className) ? className[i] : className,
              )}
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
