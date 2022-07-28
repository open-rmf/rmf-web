import { TextField } from '@mui/material';
import React from 'react';
import { ConfirmationDialog, useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';

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
  const safeAsync = useAsync();
  const [creating, setCreating] = React.useState(false);
  const [username, setUsername] = React.useState('');
  const [usernameError, setUsernameError] = React.useState(false);
  const { showAlert } = React.useContext(AppControllerContext);

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
      createUser && (await safeAsync(createUser(username)));
      setCreating(false);
      setOpen && setOpen(false);
    } catch (e) {
      setCreating(false);
      showAlert('error', `Failed to create user: ${(e as Error).message}`);
    }
  };

  return (
    <ConfirmationDialog
      open={open}
      title="Create User"
      confirmText="Create"
      submitting={creating}
      onSubmit={submitForm}
      onClose={() => setOpen && setOpen(false)}
    >
      <TextField
        id="username"
        variant="outlined"
        fullWidth
        autoFocus
        margin="normal"
        label="Username"
        value={username}
        onChange={(ev) => setUsername(ev.target.value)}
        error={usernameError}
        helperText="Required"
      />
    </ConfirmationDialog>
  );
}
