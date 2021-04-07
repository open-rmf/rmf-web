import {
  createStyles,
  makeStyles,
  AppBar,
  IconButton,
  Menu,
  MenuItem,
  Theme,
  Toolbar,
  Typography,
} from '@material-ui/core';
import AccountCircle from '@material-ui/icons/AccountCircle';
import React from 'react';

export interface TitleBarProps {
  logoPath: string;
}

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    root: {
      flexGrow: 1,
    },
    logo: {
      maxHeight: 30,
      marginTop: theme.spacing(1),
      marginBottom: theme.spacing(1),
    },
    subtitle: {
      textAlign: 'right',
      flexGrow: 1,
    },
    accountButton: {
      alignSelf: 'right',
    },
  }),
);

const TitleBar = (props: TitleBarProps): JSX.Element => {
  const { logoPath } = props;
  const classes = useStyles();

  const [anchorEl, setAnchorEl] = React.useState<null | HTMLElement>(null);
  const open = Boolean(anchorEl);
  const handleMenu = (event: React.MouseEvent<HTMLElement>) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  return (
    <div className={classes.root}>
      <AppBar position="static">
        <Toolbar variant="dense">
          <img src={logoPath} alt="logo" className={classes.logo} />
          <Typography variant="caption" className={classes.subtitle}>
            Powered by OpenRMF
          </Typography>
          <div>
            <IconButton
              aria-label="account of current user"
              aria-controls="menu-appbar"
              aria-haspopup="true"
              onClick={handleMenu}
              color="inherit"
              className={classes.accountButton}
              edge="end"
            >
              <AccountCircle />
            </IconButton>
            <Menu
              id="menu-appbar"
              anchorEl={anchorEl}
              anchorOrigin={{
                vertical: 'top',
                horizontal: 'right',
              }}
              keepMounted
              transformOrigin={{
                vertical: 'top',
                horizontal: 'right',
              }}
              open={open}
              onClose={handleClose}
            >
              <MenuItem onClick={handleClose}>Logout</MenuItem>
            </Menu>
          </div>
        </Toolbar>
      </AppBar>
    </div>
  );
};

export default TitleBar;
