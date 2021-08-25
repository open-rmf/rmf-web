import {
  Card,
  CardHeader,
  Checkbox,
  FormControlLabel,
  IconButton,
  Menu,
  MenuItem,
} from '@material-ui/core';
import { makeStyles } from '@material-ui/core/styles';
import AccountIcon from '@material-ui/icons/AccountCircle';
import MoreIcon from '@material-ui/icons/MoreVert';
import { User } from 'api-client';
import React from 'react';
import { useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';

const useStyles = makeStyles((theme) => ({
  avatar: {
    color: theme.palette.type === 'light' ? theme.palette.grey[400] : theme.palette.grey[600],
    fontSize: '3em',
  },
}));

export interface UserProfileCardProps {
  user: User;
  makeAdmin?: (admin: boolean) => Promise<void> | void;
}

export function UserProfileCard({ user, makeAdmin }: UserProfileCardProps): JSX.Element {
  const classes = useStyles();
  const safeAsync = useAsync();
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const [disableAdminCheckbox, setDisableAdminCheckbox] = React.useState(false);
  const { showErrorAlert } = React.useContext(AppControllerContext);

  return (
    <Card variant="outlined">
      <CardHeader
        title={user.username}
        titleTypographyProps={{ variant: 'h5' }}
        subheader={user.is_admin ? 'Admin' : 'User'}
        avatar={<AccountIcon className={classes.avatar} />}
        action={
          <IconButton onClick={(ev) => setAnchorEl(ev.currentTarget)} aria-label="more actions">
            <MoreIcon />
          </IconButton>
        }
      />
      <Menu open={!!anchorEl} anchorEl={anchorEl} onClose={() => setAnchorEl(null)}>
        <MenuItem
          onClick={async () => {
            setDisableAdminCheckbox(true);
            try {
              makeAdmin && (await safeAsync(makeAdmin(!user.is_admin)));
            } catch (e) {
              showErrorAlert(`Failed to change admin status: ${e.message}`);
            } finally {
              setDisableAdminCheckbox(false);
            }
          }}
        >
          <FormControlLabel
            control={<Checkbox checked={user.is_admin} disabled={disableAdminCheckbox} />}
            label="Admin"
          />
        </MenuItem>
      </Menu>
    </Card>
  );
}
