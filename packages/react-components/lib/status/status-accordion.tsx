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
  Card,
  CardHeader,
  Avatar,
} from '@material-ui/core';
import FiberManualRecordIcon from '@material-ui/icons/FiberManualRecord';
import { StatusIndicator } from './index';

export interface StatusAccordionProps {
  statusIndicators: StatusIndicator;
  itemState: { [key: string]: boolean };
  severityDisplay: boolean;
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
  list: {
    padding: theme.spacing(2),
  },
  listItemIcon: {
    minWidth: '2rem',
  },
  cardHeader: {
    backgroundColor: '#f5f5f5',
  },
  cardTypography: {
    padding: theme.spacing(1),
    color: 'white',
  },
  cardTypographyWarning: {
    backgroundColor: theme.palette.warning.main,
  },
  cardTypographyGreen: {
    backgroundColor: theme.palette.success.main,
  },
}));

export const StatusAccordion = (props: StatusAccordionProps): JSX.Element => {
  const classes = useStyles();
  const { statusIndicators, itemState, severityDisplay } = props;
  // background color of status card header
  const getStatusLabel = (severityDisplay: boolean): string => {
    return severityDisplay
      ? `
        ${classes.cardTypography} ${classes.cardTypographyGreen}`
      : `${classes.cardTypography} ${classes.cardTypographyWarning}`;
  };

  // top message on the status card header
  const getStatusMessage = (severityDisplay: boolean): string => {
    return severityDisplay ? 'All systems up' : 'Some services might be down';
  };

  return (
    <Card>
      <Typography className={getStatusLabel(severityDisplay)} align="center" variant="body1">
        {getStatusMessage(severityDisplay)}
      </Typography>
      <CardHeader
        avatar={
          // TODO - replace url with process.env variable once ready for production
          <Avatar src="http://localhost:3000/favicon.ico" />
        }
        className={classes.cardHeader}
        title="RMF Services"
        subheader="Issues related to services managed by rmf"
      />
      <List className={classes.list}>
        {Object.keys(statusIndicators).map((category) => {
          return (
            <React.Fragment key={category}>
              <ListItem className={classes.item}>
                <ListItemIcon className={classes.listItemIcon}>
                  <FiberManualRecordIcon
                    fontSize="small"
                    className={itemState[category] ? classes.online : classes.error}
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
                        <ListItemIcon className={classes.listItemIcon}>
                          <FiberManualRecordIcon
                            fontSize="small"
                            className={
                              statusIndicators[category][item].state
                                ? classes.online
                                : classes.error
                            }
                          />
                        </ListItemIcon>
                        <ListItemText primary={item} />
                      </ListItem>
                    );
                  })}
                </List>
              </Collapse>
              <Divider />
            </React.Fragment>
          );
        })}
      </List>
    </Card>
  );
};
