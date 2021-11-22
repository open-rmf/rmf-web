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
import { ConfirmationDialog, Loading, useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';
import { CreateRoleDialog, CreateRoleDialogProps } from './create-role-dialog';
import { PermissionsCard, PermissionsCardProps } from './permissions-card';

const useRoleAccordionStyles = makeStyles({
  permissionsCard: {
    width: '100%',
  },
  deleteRoleButton: {
    float: 'right',
  },
});

interface RoleAccordionProps
  extends Pick<PermissionsCardProps, 'getPermissions' | 'savePermission' | 'removePermission'> {
  role: string;
  onDeleteClick?: React.MouseEventHandler;
}

function RoleAccordion({
  role,
  onDeleteClick,
  getPermissions,
  savePermission,
  removePermission,
}: RoleAccordionProps) {
  const classes = useRoleAccordionStyles();
  return (
    <Accordion TransitionProps={{ unmountOnExit: true }}>
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
              onClick={onDeleteClick}
            >
              Delete Role
            </Button>
          </Grid>
          <PermissionsCard
            className={classes.permissionsCard}
            getPermissions={getPermissions}
            savePermission={savePermission}
            removePermission={removePermission}
          />
        </Grid>
      </AccordionDetails>
    </Accordion>
  );
}

export interface RoleListCardProps extends Pick<CreateRoleDialogProps, 'createRole'> {
  getRoles?: () => Promise<string[]> | string[];
  deleteRole?: (role: string) => Promise<void> | void;
  getPermissions?: (role: string) => Promise<Permission[]> | Permission[];
  savePermission?: (role: string, permission: Permission) => Promise<void> | void;
  removePermission?: (role: string, permission: Permission) => Promise<void> | void;
}

export function RoleListCard({
  getRoles,
  deleteRole,
  getPermissions,
  savePermission,
  removePermission,
  createRole,
}: RoleListCardProps): JSX.Element {
  const safeAsync = useAsync();
  const [roles, setRoles] = React.useState<string[]>([]);
  const [loading, setLoading] = React.useState(true);
  const [openDialog, setOpenDialog] = React.useState(false);
  const [selectedDeleteRole, setSelectedDeleteRole] = React.useState<string | null>(null);
  const [deleting, setDeleting] = React.useState(false);
  const { showErrorAlert } = React.useContext(AppControllerContext);

  const refresh = React.useCallback(async () => {
    if (!getRoles) return;
    setLoading(true);
    try {
      const newRoles = await safeAsync(getRoles());
      setRoles(newRoles.sort());
    } catch (e) {
      showErrorAlert(`Failed to get roles: ${e.message}`);
    } finally {
      setLoading(false);
    }
  }, [getRoles, showErrorAlert, safeAsync]);

  React.useEffect(() => {
    refresh();
  }, [refresh]);

  const getRolePermissions = React.useMemo(
    () => roles.map((r) => getPermissions && (() => getPermissions(r))),
    [roles, getPermissions],
  );

  const saveRolePermissions = React.useMemo(
    () => roles.map((r) => savePermission && ((p: Permission) => savePermission(r, p))),
    [roles, savePermission],
  );

  const removeRolePermissions = React.useMemo(
    () => roles.map((r) => removePermission && ((p: Permission) => removePermission(r, p))),
    [roles, removePermission],
  );

  const handleRoleDelete = React.useMemo(
    () =>
      roles.map((r) => () => {
        setDeleting(false);
        setSelectedDeleteRole(r);
      }),
    [roles],
  );

  return (
    <Card variant="outlined">
      <CardHeader
        title="Roles"
        titleTypographyProps={{ variant: 'h5' }}
        avatar={<SecurityIcon />}
        action={
          <IconButton onClick={() => setOpenDialog(true)} aria-label="create role">
            <AddIcon fontSize="large" />
          </IconButton>
        }
      />
      <Divider />
      <Loading loading={loading} size="50px">
        {roles.map((r, i) => (
          <RoleAccordion
            key={r}
            role={r}
            getPermissions={getRolePermissions[i]}
            savePermission={saveRolePermissions[i]}
            removePermission={removeRolePermissions[i]}
            onDeleteClick={handleRoleDelete[i]}
          />
        ))}
        {roles.length === 0 && <div style={{ height: 100 }} />}
      </Loading>
      {openDialog && (
        <CreateRoleDialog
          open={openDialog}
          setOpen={setOpenDialog}
          createRole={
            createRole &&
            (async (r) => {
              await createRole(r);
              refresh();
            })
          }
        />
      )}
      {selectedDeleteRole && (
        <ConfirmationDialog
          open={!!selectedDeleteRole}
          title="Confirm Delete"
          submitting={!!deleting}
          onClose={() => setSelectedDeleteRole(null)}
          onSubmit={async () => {
            try {
              setDeleting(true);
              deleteRole && (await safeAsync(deleteRole(selectedDeleteRole)));
              refresh();
            } catch (e) {
              showErrorAlert(`Failed to delete user: ${e.message}`);
            } finally {
              setSelectedDeleteRole(null);
            }
          }}
        >
          <Typography>{`Are you sure you want to delete "${selectedDeleteRole}"?`}</Typography>
        </ConfirmationDialog>
      )}
    </Card>
  );
}
