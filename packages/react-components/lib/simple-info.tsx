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

const useStyles = makeStyles((theme) => ({
  container: {
    display: 'table',
    borderCollapse: 'collapse',
    width: '100%',
  },
  tableRow: {
    display: 'table-row',
  },
  displayName: {
    display: 'table-cell',
    borderBottom: '1px solid',
    borderBottomColor: theme.palette.divider,
    borderTop: '1px solid',
    borderTopColor: theme.palette.divider,
    background: theme.palette.action.hover,
    padding: theme.spacing(0.25, 2),
    width: '30%',
  },
  value: {
    display: 'table-cell',
    textAlign: 'end',
    borderBottom: '1px solid',
    borderBottomColor: theme.palette.divider,
    borderTop: '1px solid',
    borderTopColor: theme.palette.divider,
    padding: theme.spacing(0.25, 2),
  },
  arrayListItem: {
    justifyContent: 'flex-end',
  },
  arrayItemValue: {
    textAlign: 'end',
  },
  disabled: {
    color: theme.palette.action.disabled,
  },
}));

export interface SimpleInfoProps extends React.HTMLProps<HTMLDivElement> {
  infoData: SimpleInfoData[];
}

export const SimpleInfo = (props: SimpleInfoProps): JSX.Element => {
  const { infoData, ...otherProps } = props;
  const classes = useStyles();

  const renderPrimitive = ({
    name,
    value,
    className,
    disabled,
  }: SimpleInfoData<DataValueTypePrimitive>) => (
    <>
      <Typography className={classes.displayName} variant="body1">
        {name}
      </Typography>
      <Typography
        variant="body1"
        className={joinClasses(classes.value, disabled ? classes.disabled : undefined, className)}
      >
        {value}
      </Typography>
    </>
  );

  const renderArray = ({
    name,
    value,
    className,
    disabled,
  }: SimpleInfoData<DataValueTypeArray>) => (
    <>
      <Typography className={classes.displayName} variant="body1">
        {name}
      </Typography>
      <List className={classes.value} dense>
        {value.map((item, i) => (
          <ListItem key={i} className={classes.arrayListItem}>
            <Typography
              variant="body1"
              className={joinClasses(
                classes.arrayItemValue,
                disabled ? classes.disabled : undefined,
                Array.isArray(className) ? className[i] : className,
              )}
            >
              {item}
            </Typography>
          </ListItem>
        ))}
      </List>
    </>
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
    <div {...otherProps}>
      <div className={classes.container} role="table">
        {infoData.map((item) => (
          <React.Fragment key={item.name}>
            <div className={classes.tableRow} role="row">
              {renderLine(item)}
            </div>
          </React.Fragment>
        ))}
      </div>
    </div>
  );
};

export default SimpleInfoProps;
