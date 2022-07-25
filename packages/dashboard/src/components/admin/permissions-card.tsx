import {
  Button,
  IconButton,
  Paper,
  PaperProps,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Toolbar,
  Typography,
} from '@mui/material';
import AddIcon from '@mui/icons-material/AddCircle';
import DeleteIcon from '@mui/icons-material/Delete';
import { Permission } from 'api-client';
import React from 'react';
import { styled } from '@mui/material';
import { Loading, useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';
import { getActionText } from '../permissions';
import { AddPermissionDialog, AddPermissionDialogProps } from './add-permission-dialog';

const prefix = 'permissions-card';
const classes = {
  title: `${prefix}-title`,
  tableContainer: `${prefix}-table-container`,
  controlsButton: `${prefix}-controls-button`,
};
const StyledPaper = styled((props: PaperProps) => <Paper {...props} />)(({ theme }) => ({
  [`& .${classes.title}`]: {
    flex: '1 1 100%',
  },
  [`& .${classes.tableContainer}`]: {
    marginLeft: theme.spacing(4),
    marginRight: theme.spacing(4),
    width: 'auto',
  },
  [`& .${classes.controlsButton}`]: {
    float: 'right',
  },
}));

export interface PermissionsCardProps
  extends PaperProps,
    Pick<AddPermissionDialogProps, 'savePermission'> {
  getPermissions?: () => Promise<Permission[]> | Permission[];
  removePermission?: (permission: Permission) => Promise<void> | void;
}

export function PermissionsCard({
  getPermissions,
  savePermission,
  removePermission,
  ...otherProps
}: PermissionsCardProps): JSX.Element {
  const safeAsync = useAsync();
  const [loading, setLoading] = React.useState(false);
  const [permissions, setPermissions] = React.useState<Permission[]>([]);
  const [openDialog, setOpenDialog] = React.useState(false);
  const { showAlert } = React.useContext(AppControllerContext);

  const refresh = React.useCallback(async () => {
    if (!getPermissions) return;
    setLoading(true);
    try {
      const newPermissions = await safeAsync(getPermissions());
      // sort by action first, then by authorization group
      newPermissions.sort((a, b) => {
        if (a.action < b.action) return -1;
        if (a.action > b.action) return 1;
        if (a.authz_grp < b.authz_grp) return -1;
        if (a.authz_grp > b.authz_grp) return 1;
        return 0;
      });
      setPermissions(newPermissions);
    } catch (e) {
      showAlert('error', `Failed to get permissions: ${(e as Error).message}`);
    } finally {
      setLoading(false);
    }
  }, [getPermissions, showAlert, safeAsync]);

  React.useEffect(() => {
    refresh();
  }, [refresh]);

  return (
    <StyledPaper elevation={0} {...otherProps}>
      <Toolbar>
        <Typography variant="h6" className={classes.title}>
          Permissions
        </Typography>
        <IconButton onClick={() => setOpenDialog(true)} aria-label="add permission">
          <AddIcon fontSize="large" />
        </IconButton>
      </Toolbar>
      <Loading loading={loading}>
        <TableContainer id="permission-table" className={classes.tableContainer}>
          <Table>
            <TableHead>
              <TableRow>
                <TableCell>Action</TableCell>
                <TableCell>Authorization Group</TableCell>
                <TableCell></TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {permissions.map((p, idx) => (
                <TableRow key={idx}>
                  <TableCell>{getActionText(p.action)}</TableCell>
                  <TableCell>{p.authz_grp}</TableCell>
                  <TableCell>
                    <Button
                      variant="contained"
                      color="secondary"
                      startIcon={<DeleteIcon />}
                      className={classes.controlsButton}
                      onClick={
                        removePermission &&
                        (async () => {
                          try {
                            await removePermission(p);
                            refresh();
                          } catch (e) {
                            showAlert(
                              'error',
                              `Failed to remove permission: ${(e as Error).message}`,
                            );
                          }
                        })
                      }
                    >
                      Remove
                    </Button>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </Loading>
      {openDialog && (
        <AddPermissionDialog
          open={openDialog}
          setOpen={setOpenDialog}
          savePermission={
            savePermission &&
            (async (p) => {
              await savePermission(p);
              refresh();
            })
          }
        />
      )}
    </StyledPaper>
  );
}
