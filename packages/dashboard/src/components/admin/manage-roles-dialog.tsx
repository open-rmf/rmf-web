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
  styled,
} from '@mui/material';
import SecurityIcon from '@mui/icons-material/Security';
import React from 'react';
import { Loading, TransferList, useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';

const prefix = 'manage-roles-dialog';
const classes = {
  action: `${prefix}-action`,
  list: `${prefix}-list`,
  dialogContent: `${prefix}-content`,
  dialogButton: `${prefix}-button`,
};
const StyledCard = styled((props: CardProps) => <Card {...props} />)(({ theme }) => ({
  [`& .${classes.action}`]: {
    margin: 0,
  },
  [`& .${classes.list}`]: {
    paddingLeft: theme.spacing(1),
    paddingRight: theme.spacing(1),
  },
  [`& .${classes.dialogContent}`]: {
    height: '50vh',
  },
  [`& .${classes.dialogButton}`]: {
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
  const safeAsync = useAsync();
  const [availableRoles, setAvailableRoles] = React.useState<string[]>([]);
  const [assignedRoles, setAssignedRoles] = React.useState<string[]>([]);
  const [loading, setLoading] = React.useState(false);
  const [saving, setSaving] = React.useState(false);
  const { showAlert } = React.useContext(AppControllerContext);

  React.useEffect(() => {
    if (!open || !getAllRoles) return;
    setLoading(true);
    (async () => {
      try {
        const allRoles = await safeAsync(getAllRoles());
        setAvailableRoles(allRoles.filter((r) => defaultAssignedRoles.indexOf(r) === -1).sort());
        setAssignedRoles(defaultAssignedRoles.sort());
      } catch (e) {
        showAlert('error', `Failed to get roles: ${(e as Error).message}`);
      } finally {
        setLoading(false);
      }
    })();
  }, [open, getAllRoles, defaultAssignedRoles, showAlert, safeAsync]);

  const handleOkClick = () => {
    setSaving(true);
    (async () => {
      try {
        saveRoles && (await safeAsync(saveRoles(assignedRoles)));
        setSaving(false);
        setOpen && setOpen(false);
      } catch (e) {
        setSaving(false);
        showAlert('error', `Failed to save roles: ${(e as Error).message}`);
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
  const [openDialog, setOpenDialog] = React.useState(false);

  return (
    <StyledCard variant="outlined" {...otherProps}>
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
    </StyledCard>
  );
}
