import {
  IconButton,
  Divider,
  Drawer,
  DrawerProps,
  FormControl,
  FormControlLabel,
  FormLabel,
  makeStyles,
  Radio,
  RadioGroup,
  useMediaQuery,
  Typography,
  Grid,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import React from 'react';
import { Settings, TrajectoryAnimation, TrajectoryDiameter, TrajectoryColor } from '../settings';

export interface SettingsDrawerProps extends DrawerProps {
  settings: Readonly<Settings>;
  onSettingsChange(settings: Settings): void;
  handleCloseButton: React.Dispatch<React.SetStateAction<boolean>>;
}

export default function SettingsDrawer(props: SettingsDrawerProps): React.ReactElement {
  const classes = useStyles();
  const { settings, onSettingsChange, handleCloseButton, ...otherProps } = props;
  const { trajectoryAnimation, trajectoryDiameter, trajectoryColor } = settings;

  const trajAnimsText = React.useMemo(
    () => Object.keys(TrajectoryAnimation).slice(Object.keys(TrajectoryAnimation).length * 0.5),
    [],
  );

  const trajDiameterText: { [key: string]: string } = {
    [TrajectoryDiameter.FixSize.toString()]: 'Fix Size',
    [TrajectoryDiameter.RobotSize.toString()]: 'Robot Size',
  };

  const trajColorText: { [key: string]: string } = {
    [TrajectoryColor.Theme.toString()]: 'Theme',
    [TrajectoryColor.RobotColor.toString()]: 'Robot Color',
    [TrajectoryColor.Shades.toString()]: 'Shades',
  };

  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10') ? 'bottom' : 'right';

  function handleTrajectoryAnimationChange(ev: React.ChangeEvent<HTMLInputElement>): void {
    const newSettings: Settings = { ...settings, trajectoryAnimation: Number(ev.target.value) };
    onSettingsChange && onSettingsChange(newSettings);
  }

  function handleTrajectoryDiameterChange(ev: React.ChangeEvent<HTMLInputElement>): void {
    const newSettings: Settings = { ...settings, trajectoryDiameter: Number(ev.target.value) };
    onSettingsChange && onSettingsChange(newSettings);
  }

  function handleTrajectorColorChange(ev: React.ChangeEvent<HTMLInputElement>): void {
    const newSettings: Settings = { ...settings, trajectoryColor: Number(ev.target.value) };
    onSettingsChange && onSettingsChange(newSettings);
  }

  return (
    <Drawer PaperProps={{ className: classes.drawer }} anchor={drawerAnchor} {...otherProps}>
      <Grid container alignItems="center">
        <Grid item className={classes.heading}>
          <Typography variant="h6">Settings</Typography>
        </Grid>
        <Grid item>
          <IconButton
            id="closeDrawerButton"
            className={classes.button}
            onClick={() => handleCloseButton(false)}
          >
            <CloseIcon />
          </IconButton>
        </Grid>
      </Grid>
      <FormControl component="fieldset">
        <FormLabel component="legend" className={classes.legendLabel}>
          Trajectory Animation
        </FormLabel>
        <Divider />
        <RadioGroup
          className={classes.trajGroup}
          value={trajectoryAnimation}
          onChange={handleTrajectoryAnimationChange}
        >
          {trajAnimsText.map((text, i) => (
            <FormControlLabel
              key={i}
              className={classes.flexBasis}
              value={i}
              control={<Radio />}
              label={text}
              name={text}
            />
          ))}
        </RadioGroup>
        <FormLabel component="legend" className={classes.legendLabel}>
          Trajectory Diameter
        </FormLabel>
        <Divider />
        <RadioGroup
          className={classes.trajGroup}
          value={trajectoryDiameter}
          onChange={handleTrajectoryDiameterChange}
        >
          {Object.keys(trajDiameterText).map((key, i) => (
            <FormControlLabel
              key={i}
              className={classes.flexBasis}
              value={i}
              control={<Radio />}
              label={trajDiameterText[key]}
              name={trajDiameterText[key]}
            />
          ))}
        </RadioGroup>
        <FormLabel component="legend" className={classes.legendLabel}>
          Trajectory Color
        </FormLabel>
        <Divider />
        <RadioGroup
          className={classes.trajGroup}
          value={trajectoryColor}
          onChange={handleTrajectorColorChange}
        >
          {Object.keys(trajColorText).map((key, i) => (
            <FormControlLabel
              key={i}
              className={classes.flexBasis}
              value={i}
              control={<Radio />}
              label={trajColorText[key]}
              name={trajColorText[key]}
            />
          ))}
        </RadioGroup>
      </FormControl>
    </Drawer>
  );
}

const useStyles = makeStyles((theme) => ({
  drawer: {
    '@media (min-aspect-ratio: 8/10)': {
      width: 300,
    },
    '@media (max-aspect-ratio: 8/10)': {
      width: '100%',
    },
  },
  legendLabel: {
    '@media (min-aspect-ratio: 8/10)': {
      fontSize: theme.typography.h6.fontSize,
      paddingBottom: theme.spacing(1),
      textAlign: 'center',
    },
    '@media (max-aspect-ratio: 8/10)': {
      fontSize: theme.typography.h6.fontSize,
      padding: theme.spacing(1),
      textAlign: 'center',
    },
  },
  trajGroup: {
    '@media (min-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(8),
    },
    '@media (max-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(8),
    },
  },
  flexBasis: {
    flexBasis: '40%',
  },
  heading: {
    margin: '0 auto 0 calc(50% - 3rem)',
  },
  button: {
    width: '3rem',
  },
}));
