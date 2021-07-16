import { TextField } from '@material-ui/core';
import React from 'react';
import { ConfirmationDialog } from 'react-components';

export interface CreateUserDialogProps {
  open: boolean;
  setOpen?: (open: boolean) => void;
  createUser?: (username: string) => Promise<void> | void;
}

export function CreateUserDialog({
  open,
  setOpen,
  createUser,
}: CreateUserDialogProps): JSX.Element {
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
    setOpen && setOpen(false);
  };

  return (
    <ConfirmationDialog
      open={open}
      title="Create User"
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
