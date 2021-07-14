import { Card, CardHeader, Divider, List, ListItem, ListItemText } from '@material-ui/core';
import { makeStyles } from '@material-ui/core/styles';
import SecurityIcon from '@material-ui/icons/Security';
import React from 'react';

const useStyles = makeStyles((theme) => ({
  roleList: {
    paddingLeft: theme.spacing(1),
    paddingRight: theme.spacing(1),
  },
  action: {
    margin: 0,
  },
}));

export interface RoleListCardProps {
  roles: string[];
  action?: React.ReactNode;
}

export function RoleListCard({ roles, action = null }: RoleListCardProps): JSX.Element {
  const classes = useStyles();

  return (
    <Card variant="outlined">
      <CardHeader
        title="Roles"
        titleTypographyProps={{ variant: 'h5' }}
        avatar={<SecurityIcon />}
        action={action}
        classes={{ action: classes.action }}
      />
      <Divider />
      <List dense className={classes.roleList}>
        {roles.map((r) => (
          <ListItem key={r}>
            <ListItemText>{r}</ListItemText>
          </ListItem>
        ))}
      </List>
    </Card>
  );
}
