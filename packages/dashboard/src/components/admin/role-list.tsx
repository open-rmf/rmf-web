import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Button,
  Card,
  CardHeader,
  Divider,
  Grid,
  Typography,
} from '@material-ui/core';
import { makeStyles } from '@material-ui/core/styles';
import DeleteIcon from '@material-ui/icons/Delete';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import SecurityIcon from '@material-ui/icons/Security';
import { Permission } from 'api-client';
import React from 'react';
import { Loading } from 'react-components';
import { PermissionsCard, PermissionsCardProps } from './permissions-card';

const useRoleAccordionStyles = makeStyles({
  permissionsCard: {
    width: '100%',
  },
  deleteRoleButton: {
    float: 'right',
  },
});

interface RoleAccordionProps extends Pick<PermissionsCardProps, 'savePermission'> {
  role: string;
  getPermissions?: (role: string) => Promise<Permission[]> | Permission[];
}

function RoleAccordion({ role, getPermissions, savePermission }: RoleAccordionProps) {
  const classes = useRoleAccordionStyles();
  const [permissions, setPermissions] = React.useState<Permission[]>([]);
  const [loading, setLoading] = React.useState(false);
  const [expanded, setExpanded] = React.useState(false);

  return (
    <Accordion expanded={expanded}>
      <AccordionSummary
        expandIcon={<ExpandMoreIcon />}
        onClick={async () => {
          setExpanded((prev) => !prev);
          if (expanded) {
            return;
          }
          setLoading(true);
          getPermissions && setPermissions(await getPermissions(role));
          setLoading(false);
        }}
      >
        <Typography>{role}</Typography>
      </AccordionSummary>
      <AccordionDetails>
        <Grid container direction="column" wrap="nowrap">
          <Grid item>
            <Button
              variant="contained"
              color="secondary"
              startIcon={<DeleteIcon />}
              className={classes.deleteRoleButton}
            >
              Delete Role
            </Button>
          </Grid>
          <Loading loading={loading}>
            <PermissionsCard
              className={classes.permissionsCard}
              permissions={permissions}
              savePermission={savePermission}
            />
          </Loading>
        </Grid>
      </AccordionDetails>
    </Accordion>
  );
}

export interface RoleListCardProps
  extends Pick<RoleAccordionProps, 'savePermission' | 'getPermissions'> {
  roles: string[];
}

export function RoleListCard({
  roles,
  getPermissions,
  savePermission,
}: RoleListCardProps): JSX.Element {
  roles.sort();

  return (
    <Card variant="outlined">
      <CardHeader
        title="Roles"
        titleTypographyProps={{ variant: 'h5' }}
        avatar={<SecurityIcon />}
      />
      <Divider />
      {roles.map((r) => (
        <RoleAccordion
          key={r}
          role={r}
          getPermissions={getPermissions}
          savePermission={savePermission}
        />
      ))}
    </Card>
  );
}
