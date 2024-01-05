import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Button,
  Card,
  CardHeader,
  CardProps,
  Divider,
  Grid,
  IconButton,
  Typography,
  styled,
} from '@mui/material';
import AddIcon from '@mui/icons-material/AddCircle';
import DeleteIcon from '@mui/icons-material/Delete';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import SecurityIcon from '@mui/icons-material/Security';
import { Permission } from 'api-client';
import React from 'react';
import { ConfirmationDialog, Loading, useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';
import { CreateRoleDialog, CreateRoleDialogProps } from './create-role-dialog';
import { PermissionsCard, PermissionsCardProps } from './permissions-card';

const prefix = 'role-list-card';
const classes = {
  permissionsCard: `${prefix}-permissionscard`,
  deleteRoleButton: `${prefix}-deleterolebutton`,
};
const StyledCard = styled((props: CardProps) => <Card {...props} />)(() => ({
  [`& .${classes.permissionsCard}`]: {
    width: '100%',
  },
  [`& .${classes.deleteRoleButton}`]: {
    float: 'right',
  },
}));

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
          <Grid item>
            <PermissionsCard
              className={classes.permissionsCard}
              getPermissions={getPermissions}
              savePermission={savePermission}
              removePermission={removePermission}
            />
          </Grid>
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
  const { showAlert } = React.useContext(AppControllerContext);

  const refresh = React.useCallback(async () => {
    if (!getRoles) return;
    setLoading(true);
    try {
      const newRoles = await safeAsync(getRoles());
      setRoles(newRoles.sort());
    } catch (e) {
      showAlert('error', `Failed to get roles: ${(e as Error).message}`);
    } finally {
      setLoading(false);
    }
  }, [getRoles, showAlert, safeAsync]);

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
    <StyledCard variant="outlined">
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
              showAlert('error', `Failed to delete user: ${(e as Error).message}`);
            } finally {
              setSelectedDeleteRole(null);
            }
          }}
        >
          <Typography>{`Are you sure you want to delete "${selectedDeleteRole}"?`}</Typography>
        </ConfirmationDialog>
      )}
    </StyledCard>
  );
}
