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
import React from 'react';
import { useAsync } from 'react-components';
import { UserProfile } from '../auth';

const useStyles = makeStyles((theme) => ({
  avatar: {
    color: theme.palette.type === 'light' ? theme.palette.grey[400] : theme.palette.grey[600],
    fontSize: '3em',
  },
}));

export interface UserProfileCardProps {
  profile: UserProfile;
  makeAdmin?: (admin: boolean) => Promise<void> | void;
}

export function UserProfileCard({ profile, makeAdmin }: UserProfileCardProps): JSX.Element {
  const classes = useStyles();
  const safeAsync = useAsync();
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const [disableAdminCheckbox, setDisableAdminCheckbox] = React.useState(false);

  return (
    <Card variant="outlined">
      <CardHeader
        title={profile.username}
        titleTypographyProps={{ variant: 'h5' }}
        subheader={profile.is_admin ? 'Admin' : 'User'}
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
            makeAdmin && (await safeAsync(makeAdmin(!profile.is_admin)));
            setDisableAdminCheckbox(false);
          }}
        >
          <FormControlLabel
            control={<Checkbox checked={profile.is_admin} disabled={disableAdminCheckbox} />}
            label="Admin"
          />
        </MenuItem>
      </Menu>
    </Card>
  );
}
