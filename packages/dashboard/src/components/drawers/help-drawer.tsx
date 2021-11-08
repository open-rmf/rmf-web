import {
  Checkbox,
  Divider,
  Drawer,
  DrawerProps,
  IconButton,
  Typography,
  useMediaQuery,
  styled,
} from '@mui/material';
import BugReportIcon from '@mui/icons-material/BugReport';
import DirectionsIcon from '@mui/icons-material/Directions';
import React from 'react';
import { AppControllerContext, TooltipsContext } from '../app-contexts';
import DrawerHeader from './drawer-header';

export interface HotKeysDrawerProps extends DrawerProps {
  handleCloseButton(): void;
  setShowHotkeyDialog(): void;
  showTour(): void;
}

const prefix = 'help-drawer';
const classes = {
  detailLine: `${prefix}-detail-line`,
  detail: `${prefix}-detail`,
  drawer: `${prefix}-root`,
};
const StyledDrawer = styled((props: DrawerProps) => <Drawer {...props} />)(({ theme }) => ({
  [`& .${classes.detailLine}`]: {
    display: 'inline-flex',
    padding: theme.spacing(0.5),
    width: '100%',
    '& button': {
      padding: '0 5px 0 0',
    },
    '&:hover': {
      backgroundColor: 'rgba(0, 0, 0, 0.04)',
    },
    cursor: 'pointer',
  },
  [`& .${classes.detail}`]: {
    display: 'flex',
    flexFlow: 'column',
    padding: '1rem',
  },
  [`&.${classes.drawer}`]: {
    '@media (min-aspect-ratio: 8/10)': {
      width: 300,
    },
    '@media (max-aspect-ratio: 8/10)': {
      width: '100%',
    },
  },
}));

export default function HelpDrawer(props: HotKeysDrawerProps): React.ReactElement {
  const { handleCloseButton, setShowHotkeyDialog, showTour, ...otherProps } = props;
  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10)') ? 'bottom' : 'right';
  const modalProp = {
    disableEnforceFocus: true,
  };
  const { showTooltips } = React.useContext(TooltipsContext);
  const { toggleTooltips } = React.useContext(AppControllerContext);

  return (
    <StyledDrawer
      PaperProps={{ className: classes.drawer }}
      anchor={drawerAnchor}
      ModalProps={modalProp}
      {...otherProps}
    >
      <DrawerHeader handleCloseButton={handleCloseButton} title={'Help'} />
      <div className={classes.detail} id="help-drawer-options">
        <div
          className={classes.detailLine}
          onClick={() => {
            setShowHotkeyDialog();
            handleCloseButton();
          }}
        >
          <IconButton id="show-hotkeys-btn" aria-label="close-help" color="inherit">
            <DirectionsIcon />
          </IconButton>
          <Typography variant="h5"> Hotkeys </Typography>
        </div>
        <Divider />

        <div className={classes.detailLine}>
          <IconButton id="show-hotkeys-btn" color="inherit">
            <BugReportIcon />
          </IconButton>
          <Typography variant="h5"> Report an error </Typography>
        </div>
        <Divider />

        <div className={classes.detailLine}>
          <Checkbox
            style={{ padding: '0 5px 0 2px' }}
            checked={showTooltips}
            size="small"
            color="primary"
            inputProps={{ 'aria-label': 'tooltip checkbox' }}
            onChange={(event) => {
              toggleTooltips();
              localStorage.setItem('dashboardTooltips', event.target.checked.toString());
            }}
          />
          <Typography variant="h5"> Toggle tooltips </Typography>
        </div>
        <Divider />
      </div>
    </StyledDrawer>
  );
}
