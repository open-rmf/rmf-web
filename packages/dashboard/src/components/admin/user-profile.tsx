import {
  Card,
  CardProps,
  CardHeader,
  Checkbox,
  FormControlLabel,
  IconButton,
  Menu,
  MenuItem,
  styled,
} from '@mui/material';
import AccountIcon from '@mui/icons-material/AccountCircle';
import MoreIcon from '@mui/icons-material/MoreVert';
import { User } from 'api-client';
import React from 'react';
import { useAsync } from 'react-components';
import { AppControllerContext } from '../app-contexts';

const classes = {
  avatar: 'user-profile-action',
};
const StyledCard = styled((props: CardProps) => <Card {...props} />)(({ theme }) => ({
  [`& .${classes.avatar}`]: {
    color: theme.palette.mode === 'light' ? theme.palette.grey[400] : theme.palette.grey[600],
    fontSize: '3em',
  },
}));

export interface UserProfileCardProps {
  user: User;
  makeAdmin?: (admin: boolean) => Promise<void> | void;
}

export function UserProfileCard({ user, makeAdmin }: UserProfileCardProps): JSX.Element {
  const safeAsync = useAsync();
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const [disableAdminCheckbox, setDisableAdminCheckbox] = React.useState(false);
  const { showAlert } = React.useContext(AppControllerContext);

  return (
    <StyledCard variant="outlined">
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
              showAlert('error', `Failed to change admin status: ${(e as Error).message}`);
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
    </StyledCard>
  );
}
