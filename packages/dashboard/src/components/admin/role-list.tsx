import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Card,
  CardHeader,
  Divider,
  Typography,
} from '@material-ui/core';
import { makeStyles } from '@material-ui/core/styles';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import SecurityIcon from '@material-ui/icons/Security';
import React from 'react';

const useStyles = makeStyles((theme) => ({}));

export interface RoleListCardProps {
  roles: string[];
  action?: React.ReactNode;
  onRoleClick?: React.MouseEventHandler;
}

export function RoleListCard({ roles, onRoleClick }: RoleListCardProps): JSX.Element {
  const classes = useStyles();

  return (
    <Card variant="outlined">
      <CardHeader
        title="Roles"
        titleTypographyProps={{ variant: 'h5' }}
        avatar={<SecurityIcon />}
      />
      <Divider />
      {roles.map((r) => (
        <Accordion key={r}>
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Typography variant="body1">{r}</Typography>
          </AccordionSummary>
          <AccordionDetails>
            <Typography>TODO: Permissions list</Typography>
          </AccordionDetails>
        </Accordion>
      ))}
    </Card>
  );
}
