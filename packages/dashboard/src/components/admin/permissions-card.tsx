import {
  Button,
  IconButton,
  makeStyles,
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
} from '@material-ui/core';
import AddIcon from '@material-ui/icons/AddCircle';
import DeleteIcon from '@material-ui/icons/Delete';
import { Permission } from 'api-client';
import React from 'react';
import { ErrorSnackbar, Loading } from '../../../../react-components/dist';
import { getActionText } from '../permissions';
import { AddPermissionDialog, AddPermissionDialogProps } from './add-permission-dialog';

const useStyles = makeStyles((theme) => ({
  title: {
    flex: '1 1 100%',
  },
  tableContainer: {
    marginLeft: theme.spacing(4),
    marginRight: theme.spacing(4),
    width: 'auto',
  },
  controlsButton: {
    float: 'right',
  },
}));

export interface PermissionsCardProps
  extends PaperProps,
    Pick<AddPermissionDialogProps, 'savePermission'> {
  getPermissions?: () => Promise<Permission[]> | Permission[];
  onRemovePermissionClick?: (ev: React.MouseEvent, permission: Permission) => void;
}

export function PermissionsCard({
  getPermissions,
  savePermission,
  onRemovePermissionClick,
  ...otherProps
}: PermissionsCardProps): JSX.Element {
  const classes = useStyles();
  const [loading, setLoading] = React.useState(false);
  const [permissions, setPermissions] = React.useState<Permission[]>([]);
  const [openDialog, setOpenDialog] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [errorMessage, setErrorMessage] = React.useState('');

  React.useEffect(() => {
    if (!getPermissions) return;
    let cancel = false;
    setLoading(true);
    (async () => {
      try {
        const newPermissions = await getPermissions();
        if (cancel) return;
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
        setErrorMessage(`Failed to get permissions: ${e.message}`);
        setOpenSnackbar(true);
      } finally {
        setLoading(false);
      }
    })();
    return () => {
      cancel = true;
    };
  }, [getPermissions]);

  return (
    <Paper elevation={0} {...otherProps}>
      <Toolbar>
        <Typography variant="h6" className={classes.title}>
          Permissions
        </Typography>
        <IconButton onClick={() => setOpenDialog(true)} aria-label="add permission">
          <AddIcon fontSize="large" color="primary" />
        </IconButton>
      </Toolbar>
      <Loading loading={loading}>
        <TableContainer className={classes.tableContainer}>
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
                      onClick={(ev) => onRemovePermissionClick && onRemovePermissionClick(ev, p)}
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
      <AddPermissionDialog
        open={openDialog}
        setOpen={setOpenDialog}
        savePermission={savePermission}
      />
      <ErrorSnackbar
        open={openSnackbar}
        message={errorMessage}
        onClose={() => setOpenSnackbar(false)}
      />
    </Paper>
  );
}
