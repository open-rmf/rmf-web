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
import { Settings, TrajectoryAnimation } from '../settings';

export interface SettingsDrawerProps extends DrawerProps {
  settings: Readonly<Settings>;
  onSettingsChange(settings: Settings): void;
}

export default function SettingsDrawer(props: SettingsDrawerProps): React.ReactElement {
  const classes = useStyles();
  const { settings, onSettingsChange, ...otherProps } = props;
  const { trajectoryAnimation } = settings;

  const trajAnimsText = React.useMemo(
    () => Object.keys(TrajectoryAnimation).slice(Object.keys(TrajectoryAnimation).length * 0.5),
    [],
  );

  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10)') ? 'bottom' : 'right';

  function handleTrajectoryAnimationChange(ev: React.ChangeEvent<HTMLInputElement>): void {
    const newSettings: Settings = { ...settings, trajectoryAnimation: Number(ev.target.value) };
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
