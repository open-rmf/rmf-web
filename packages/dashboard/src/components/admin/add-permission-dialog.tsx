import {
  Dialog,
  DialogContent,
  DialogProps,
  DialogTitle,
  makeStyles,
  MenuItem,
  TextField,
} from '@material-ui/core';
import { Permission } from 'api-client';
import React from 'react';
import { ConfirmDialogActions } from 'react-components';
import { getActionText, RmfAction } from '../permissions';

const useConfirmDialogActionsStyles = makeStyles({
  button: {
    width: 80,
  },
});

const useStyles = makeStyles((theme) => ({
  form: {
    '&>:not(:first-child)': {
      marginTop: theme.spacing(1),
    },
  },
}));

export interface AddPermissionDialogProps extends DialogProps {
  setOpen?: (open: boolean) => void;
  savePermission?: (permission: Permission) => Promise<void> | void;
}

export function AddPermissionDialog({
  setOpen,
  savePermission,
  ...otherProps
}: AddPermissionDialogProps): JSX.Element {
  const classes = useStyles();
  const confirmDialogActionsClasses = useConfirmDialogActionsStyles();
  const [action, setAction] = React.useState('');
  const [authzGrp, setAuthzGrp] = React.useState('');
  const [actionError, setActionError] = React.useState(false);
  const [authzGrpError, setAuthzGrpError] = React.useState(false);
  const [saving, setSaving] = React.useState(false);

  const validateForm = () => {
    if (!action) {
      setActionError(true);
    }
    if (!authzGrp) {
      setAuthzGrpError(true);
    }
  };

  const handleConfirm = async () => {
    validateForm();
    if (actionError || authzGrpError) {
      return;
    }
    setSaving(true);
    savePermission && (await savePermission({ action, authz_grp: authzGrp }));
    setSaving(false);
  };

  return (
    <Dialog {...otherProps}>
      <DialogTitle>Add Permission</DialogTitle>
      <DialogContent dividers>
        <form className={classes.form}>
          <TextField
            id="action-input"
            select
            variant="outlined"
            fullWidth
            label="Action"
            value={action}
            onChange={(ev) => setAction(ev.target.value)}
            error={actionError}
            helperText="Required"
          >
            {Object.values(RmfAction).map((act) => (
              <MenuItem key={act} value={act}>
                {getActionText(act)}
              </MenuItem>
            ))}
          </TextField>
          <TextField
            id="authz-grp-input"
            variant="outlined"
            fullWidth
            label="Authorization Group"
            value={authzGrp}
            onChange={(ev) => setAuthzGrp(ev.target.value)}
            error={authzGrpError}
            helperText="Required"
          />
        </form>
      </DialogContent>
      <ConfirmDialogActions
        loading={saving}
        confirmText="Save"
        classes={confirmDialogActionsClasses}
        onConfirmClick={handleConfirm}
        onCancelClick={() => setOpen && setOpen(false)}
      />
    </Dialog>
  );
}
