import { TextField } from '@material-ui/core';
import React from 'react';
import { ConfirmationDialog, ErrorSnackbar } from 'react-components';

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
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [errorMessage, setErrorMessage] = React.useState('');

  const validateForm = () => {
    let error = false;
    if (!username) {
      setUsernameError(true);
      error = true;
    } else {
      setUsernameError(false);
    }
    return !error;
  };

  const submitForm = async () => {
    if (!validateForm()) {
      return;
    }
    setCreating(true);
    try {
      createUser && (await createUser(username));
      setOpen && setOpen(false);
    } catch (e) {
      setErrorMessage(`Failed to create user: ${e.message}`);
      setOpenSnackbar(true);
    } finally {
      setCreating(false);
    }
  };

  return (
    <ConfirmationDialog
      open={open}
      title="Create User"
      confirmText="Create"
      loading={creating}
      onSubmit={submitForm}
      onCancelClick={() => setOpen && setOpen(false)}
    >
      <TextField
        id="username"
        variant="outlined"
        fullWidth
        autoFocus
        label="Username"
        value={username}
        onChange={(ev) => setUsername(ev.target.value)}
        error={usernameError}
        helperText="Required"
      />
      <ErrorSnackbar
        open={openSnackbar}
        message={errorMessage}
        onClose={() => setOpenSnackbar(false)}
      />
    </ConfirmationDialog>
  );
}
