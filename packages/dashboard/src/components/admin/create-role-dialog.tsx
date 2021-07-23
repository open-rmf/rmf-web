import { TextField } from '@material-ui/core';
import React from 'react';
import { ConfirmationDialog, useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';

export interface CreateRoleDialogProps {
  open: boolean;
  setOpen?: (open: boolean) => void;
  createRole?: (role: string) => Promise<void> | void;
}

export function CreateRoleDialog({
  open,
  setOpen,
  createRole,
}: CreateRoleDialogProps): JSX.Element {
  const safeAsync = useAsync();
  const [creating, setCreating] = React.useState(false);
  const [role, setRole] = React.useState('');
  const [roleError, setRoleError] = React.useState(false);
  const { showErrorAlert } = React.useContext(AppControllerContext);

  const validateForm = () => {
    let error = false;
    if (!role) {
      setRoleError(true);
      error = true;
    } else {
      setRoleError(false);
    }
    return !error;
  };

  const submitForm = async () => {
    if (!validateForm()) {
      return;
    }
    setCreating(true);
    try {
      createRole && (await safeAsync(createRole(role)));
      setCreating(false);
      setOpen && setOpen(false);
    } catch (e) {
      setCreating(false);
      showErrorAlert(`Failed to create role: ${e.message}`);
    }
  };

  return (
    <ConfirmationDialog
      open={open}
      title="Create Role"
      confirmText="Create"
      loading={creating}
      onSubmit={submitForm}
      onCancelClick={() => setOpen && setOpen(false)}
    >
      <TextField
        id="role"
        variant="outlined"
        fullWidth
        autoFocus
        label="Role"
        value={role}
        onChange={(ev) => setRole(ev.target.value)}
        error={roleError}
        helperText="Required"
      />
    </ConfirmationDialog>
  );
}
