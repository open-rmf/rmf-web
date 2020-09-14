import {
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
} from '@material-ui/core';
import React from 'react';
import { Settings, TrajectoryAnimation, TrajectoryDiameter, TrajectoryColor } from '../settings';

export interface SettingsDrawerProps extends DrawerProps {
  settings: Readonly<Settings>;
  onSettingsChange(settings: Settings): void;
}

export default function SettingsDrawer(props: SettingsDrawerProps): React.ReactElement {
  const classes = useStyles();
  const { settings, onSettingsChange, ...otherProps } = props;
  const { trajectoryAnimation, trajectoryDiameter, trajectoryColor } = settings;

  const trajAnimsText = React.useMemo(
    () => Object.keys(TrajectoryAnimation).slice(Object.keys(TrajectoryAnimation).length * 0.5),
    [],
  );

  const trajDiameterText = React.useMemo(() => {
    const keys = Object.keys(TrajectoryDiameter).slice(
      Object.keys(TrajectoryDiameter).length * 0.5,
    );
    return keys.map(key => {
      let text = '';
      key.split(/(?=[A-Z])/).forEach(k => {
        text += k + ' ';
      });
      return text.trim();
    });
  }, []);

  const trajColorText = React.useMemo(() => {
    const keys = Object.keys(TrajectoryColor).slice(Object.keys(TrajectoryColor).length * 0.5);
    return keys.map(key => {
      let text = '';
      key.split(/(?=[A-Z])/).forEach(k => {
        text += k + ' ';
      });
      return text.trim();
    });
  }, []);

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
          {trajDiameterText.map((text, i) => (
            <FormControlLabel
              key={i}
              className={classes.flexBasis}
              value={i}
              control={<Radio />}
              label={text}
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
          {trajColorText.map((text, i) => (
            <FormControlLabel
              key={i}
              className={classes.flexBasis}
              value={i}
              control={<Radio />}
              label={text}
            />
          ))}
        </RadioGroup>
      </FormControl>
    </Drawer>
  );
}

const useStyles = makeStyles(theme => ({
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
      padding: theme.spacing(1),
    },
    '@media (max-aspect-ratio: 8/10)': {
      fontSize: theme.typography.h6.fontSize,
      padding: theme.spacing(1),
    },
  },
  trajGroup: {
    '@media (min-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(4),
    },
    '@media (max-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(8),
    },
  },
  flexBasis: {
    flexBasis: '40%',
  },
}));
