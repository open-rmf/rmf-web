import { TextField } from '@material-ui/core';
import React from 'react';
import { ConfirmationDialog } from 'react-components';

export interface AddUserDialogProps {
  open: boolean;
  setOpen?: (open: boolean) => void;
  createUser?: (username: string) => Promise<void> | void;
}

export function AddUserDialog({ open, setOpen, createUser }: AddUserDialogProps): JSX.Element {
  const [creating, setCreating] = React.useState(false);
  const [username, setUsername] = React.useState('');
  const [usernameError, setUsernameError] = React.useState(false);

  const validateForm = () => {
    let error = false;
    if (!username) {
      setUsernameError(true);
      error = true;
    }
    return !error;
  };

  const submitForm = async () => {
    if (!validateForm()) {
      return;
    }
    setUsernameError(false);
    setCreating(true);
    createUser && (await createUser(username));
    setCreating(false);
  };

  return (
    <ConfirmationDialog
      open={open}
      title="Add User"
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
          id="username"
          variant="outlined"
          fullWidth
          label="Username"
          value={username}
          onChange={(ev) => setUsername(ev.target.value)}
          error={usernameError}
          helperText="Required"
        />
      </form>
    </ConfirmationDialog>
  );
}
