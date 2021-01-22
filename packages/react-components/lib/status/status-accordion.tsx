import React from 'react';
import {
  makeStyles,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Collapse,
  Divider,
  Typography,
} from '@material-ui/core';
import FiberManualRecordIcon from '@material-ui/icons/FiberManualRecord';
import { StatusIndicator } from './index';

export interface StatusAccordionProps {
  statusIndicators: StatusIndicator;
  itemIndicator: { [key: string]: boolean };
}

const useStyles = makeStyles((theme) => ({
  serviceIndicator: {
    height: '1.5rem',
    width: '1.5rem',
    borderRadius: '50%',
    border: 'none',
  },
  online: {
    color: theme.palette.success.main,
  },
  error: {
    color: theme.palette.error.main,
  },
  item: {
    padding: 0,
  },
  nestedList: {
    paddingLeft: theme.spacing(4),
  },
}));

export const StatusAccordion = (props: StatusAccordionProps): JSX.Element => {
  const classes = useStyles();
  const { statusIndicators, itemIndicator } = props;

  return (
    <List>
      {Object.keys(statusIndicators).map((category) => {
        return (
          <React.Fragment key={category}>
            <Divider />
            <ListItem className={classes.item}>
              <ListItemIcon>
                <FiberManualRecordIcon
                  className={itemIndicator[category] ? classes.online : classes.error}
                />
              </ListItemIcon>
              <ListItemText
                disableTypography
                primary={<Typography variant="h6">{category}</Typography>}
              />
            </ListItem>
            <Collapse in={true}>
              <List className={classes.nestedList}>
                {Object.keys(statusIndicators[category]).map((item) => {
                  return (
                    <ListItem key={item} className={classes.item}>
                      <ListItemIcon>
                        <FiberManualRecordIcon
                          className={
                            statusIndicators[category][item].state ? classes.online : classes.error
                          }
                        />
                      </ListItemIcon>
                      <ListItemText primary={item} />
                    </ListItem>
                  );
                })}
              </List>
            </Collapse>
          </React.Fragment>
        );
      })}
    </List>
  );
};
