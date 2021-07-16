import { TextField } from '@material-ui/core';
import React from 'react';
import { ConfirmationDialog } from 'react-components';

export interface AddRoleDialogProps {
  open: boolean;
  setOpen?: (open: boolean) => void;
  createRole?: (role: string) => Promise<void> | void;
}

export function AddRoleDialog({ open, setOpen, createRole }: AddRoleDialogProps): JSX.Element {
  const [creating, setCreating] = React.useState(false);
  const [role, setRole] = React.useState('');
  const [roleError, setRoleError] = React.useState(false);

  const validateForm = () => {
    let error = false;
    if (!role) {
      setRoleError(true);
      error = true;
    }
    return !error;
  };

  const submitForm = async () => {
    if (!validateForm()) {
      return;
    }
    setRoleError(false);
    setCreating(true);
    createRole && (await createRole(role));
    setCreating(false);
  };

  return (
    <ConfirmationDialog
      open={open}
      title="Add Role"
      confirmText="Create"
      loading={creating}
      onCancelClick={() => setOpen && setOpen(false)}
      onConfirmClick={submitForm}
    >
      <form
        onSubmit={(ev) => {
          ev.preventDefault();
          submitForm();
        }}
      >
        <TextField
          id="role"
          variant="outlined"
          fullWidth
          label="Role"
          value={role}
          onChange={(ev) => setRole(ev.target.value)}
          error={roleError}
          helperText="Required"
        />
      </form>
    </ConfirmationDialog>
  );
}
