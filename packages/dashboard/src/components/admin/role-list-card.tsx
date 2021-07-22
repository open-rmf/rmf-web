import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Button,
  Card,
  CardHeader,
  Divider,
  Grid,
  IconButton,
  Typography,
} from '@material-ui/core';
import { makeStyles } from '@material-ui/core/styles';
import AddIcon from '@material-ui/icons/AddCircle';
import DeleteIcon from '@material-ui/icons/Delete';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import SecurityIcon from '@material-ui/icons/Security';
import { Permission } from 'api-client';
import React from 'react';
import { ErrorSnackbar, Loading } from 'react-components';
import { CreateRoleDialog, CreateRoleDialogProps } from './create-role-dialog';
import { PermissionsCard } from './permissions-card';

const useRoleAccordionStyles = makeStyles({
  permissionsCard: {
    width: '100%',
  },
  deleteRoleButton: {
    float: 'right',
  },
});

interface RoleAccordionProps {
  role: string;
  getPermissions?: (role: string) => Promise<Permission[]> | Permission[];
  savePermission?: (role: string, permission: Permission) => Promise<void> | void;
}

function RoleAccordion({ role, getPermissions, savePermission }: RoleAccordionProps) {
  const classes = useRoleAccordionStyles();

  return (
    <Accordion>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
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
          <PermissionsCard
            className={classes.permissionsCard}
            getPermissions={getPermissions && (() => getPermissions(role))}
            savePermission={(p) => savePermission && savePermission(role, p)}
          />
        </Grid>
      </AccordionDetails>
    </Accordion>
  );
}

export interface RoleListCardProps
  extends Pick<RoleAccordionProps, 'savePermission' | 'getPermissions'>,
    Pick<CreateRoleDialogProps, 'createRole'> {
  getRoles?: () => Promise<string[]> | string[];
}

export function RoleListCard({
  getRoles,
  getPermissions,
  savePermission,
  createRole,
}: RoleListCardProps): JSX.Element {
  const [roles, setRoles] = React.useState<string[]>([]);
  const [loading, setLoading] = React.useState(true);
  const [openDialog, setOpenDialog] = React.useState(false);
  const [refresh, setRefresh] = React.useState(0);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [errorMessage, setErrorMessage] = React.useState('');
  roles.sort();

  React.useEffect(() => {
    if (!getRoles) return;
    let cancel = false;
    setLoading(true);
    (async () => {
      try {
        const newRoles = await getRoles();
        if (cancel) return;
        setRoles(newRoles);
      } catch (e) {
        setErrorMessage(`Failed to get roles: ${e.message}`);
        setOpenSnackbar(true);
      } finally {
        setLoading(false);
      }
    })();
    return () => {
      cancel = true;
    };
  }, [getRoles, refresh]);

  return (
    <Card variant="outlined">
      <CardHeader
        title="Roles"
        titleTypographyProps={{ variant: 'h5' }}
        avatar={<SecurityIcon />}
        action={
          <IconButton color="primary" onClick={() => setOpenDialog(true)} aria-label="create role">
            <AddIcon fontSize="large" />
          </IconButton>
        }
      />
      <Divider />
      <Loading loading={loading}>
        {roles.map((r) => (
          <RoleAccordion
            key={r}
            role={r}
            getPermissions={
              getPermissions &&
              (async (r) => {
                try {
                  return await getPermissions(r);
                } catch (e) {
                  setErrorMessage(`Failed to get permissions: ${e.message}`);
                  setOpenSnackbar(true);
                  return [];
                }
              })
            }
            savePermission={savePermission}
          />
        ))}
        {openDialog && (
          <CreateRoleDialog
            open={openDialog}
            setOpen={setOpenDialog}
            createRole={async (r) => {
              createRole && (await createRole(r));
              setRefresh((prev) => prev + 1);
            }}
          />
        )}
      </Loading>
      <ErrorSnackbar
        open={openSnackbar}
        message={errorMessage}
        onClose={() => setOpenSnackbar(false)}
      />
    </Card>
  );
}
