import {
  Button,
  Card,
  CardHeader,
  CardProps,
  Dialog,
  DialogActions,
  DialogContent,
  DialogProps,
  DialogTitle,
  Divider,
  List,
  ListItem,
  ListItemText,
  makeStyles,
} from '@material-ui/core';
import SecurityIcon from '@material-ui/icons/Security';
import React from 'react';
import { Loading, TransferList, useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';

const useStyles = makeStyles((theme) => ({
  action: {
    margin: 0,
  },
  list: {
    paddingLeft: theme.spacing(1),
    paddingRight: theme.spacing(1),
  },
  dialogContent: {
    height: '50vh',
  },
  dialogButton: {
    width: 100,
  },
}));

export interface ManageRolesDialogProps extends Omit<DialogProps, 'onClose'> {
  defaultAssignedRoles: string[];
  setOpen?: (open: boolean) => void;
  getAllRoles?: () => Promise<string[]>;
  saveRoles?: (roles: string[]) => Promise<void>;
}

export function ManageRolesDialog({
  defaultAssignedRoles,
  setOpen,
  getAllRoles,
  saveRoles,
  open,
  ...dialogProps
}: ManageRolesDialogProps): JSX.Element {
  const classes = useStyles();
  const safeAsync = useAsync();
  const [availableRoles, setAvailableRoles] = React.useState<string[]>([]);
  const [assignedRoles, setAssignedRoles] = React.useState<string[]>([]);
  const [loading, setLoading] = React.useState(false);
  const [saving, setSaving] = React.useState(false);
  const { showErrorAlert } = React.useContext(AppControllerContext);

  React.useEffect(() => {
    if (!open || !getAllRoles) return;
    setLoading(true);
    (async () => {
      try {
        const allRoles = await safeAsync(getAllRoles());
        setAvailableRoles(allRoles.filter((r) => defaultAssignedRoles.indexOf(r) === -1).sort());
        setAssignedRoles(defaultAssignedRoles.sort());
      } catch (e) {
        showErrorAlert(`Failed to get roles: ${e.message}`);
      } finally {
        setLoading(false);
      }
    })();
  }, [open, getAllRoles, defaultAssignedRoles, showErrorAlert, safeAsync]);

  const handleOkClick = () => {
    setSaving(true);
    (async () => {
      try {
        saveRoles && (await safeAsync(saveRoles(assignedRoles)));
        setSaving(false);
        setOpen && setOpen(false);
      } catch (e) {
        setSaving(false);
        showErrorAlert(`Failed to save roles: ${e.message}`);
      }
    })();
  };

  return (
    <Dialog
      maxWidth="md"
      fullWidth
      open={open}
      onClose={() => setOpen && setOpen(false)}
      {...dialogProps}
    >
      <DialogTitle>Manage Roles</DialogTitle>
      <DialogContent dividers className={classes.dialogContent}>
        <Loading loading={loading} size="5em">
          <TransferList
            leftItems={availableRoles}
            rightItems={assignedRoles}
            leftTitle="Available Roles"
            rightTitle="Assigned Roles"
            onTransfer={(left, right) => {
              setAvailableRoles(left.sort());
              setAssignedRoles(right.sort());
            }}
          />
        </Loading>
      </DialogContent>
      <DialogActions>
        <Button
          variant="outlined"
          color="secondary"
          aria-label="Cancel"
          className={classes.dialogButton}
          onClick={() => {
            setLoading(false);
            setOpen && setOpen(false);
          }}
          disabled={saving}
        >
          Cancel
        </Button>
        <Button
          variant="contained"
          color="primary"
          aria-label="OK"
          disabled={saving}
          className={classes.dialogButton}
          onClick={handleOkClick}
        >
          <Loading hideChildren loading={saving} size="1.5em" color="inherit">
            Save
          </Loading>
        </Button>
      </DialogActions>
    </Dialog>
  );
}

export interface ManageRolesCardProps
  extends CardProps,
    Pick<ManageRolesDialogProps, 'getAllRoles' | 'saveRoles'> {
  assignedRoles: string[];
}

export function ManageRolesCard({
  assignedRoles,
  getAllRoles,
  saveRoles,
  ...otherProps
}: ManageRolesCardProps): JSX.Element {
  const classes = useStyles();
  const [openDialog, setOpenDialog] = React.useState(false);

  return (
    <Card variant="outlined" {...otherProps}>
      <CardHeader
        title="Roles"
        titleTypographyProps={{ variant: 'h5' }}
        avatar={<SecurityIcon />}
        action={
          <Button
            variant="contained"
            color="primary"
            aria-label="Add/Remove"
            onClick={() => {
              setOpenDialog(true);
            }}
          >
            Add/Remove
          </Button>
        }
        classes={{ action: classes.action }}
      />
      <Divider />
      <List dense className={classes.list}>
        {assignedRoles.map((r) => (
          <ListItem key={r}>
            <ListItemText>{r}</ListItemText>
          </ListItem>
        ))}
      </List>
      <ManageRolesDialog
        open={openDialog}
        setOpen={setOpenDialog}
        defaultAssignedRoles={assignedRoles}
        getAllRoles={getAllRoles}
        saveRoles={saveRoles}
      />
    </Card>
  );
}
