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
  useMediaQuery
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

  const drawerAnchor = useMediaQuery("(max-width: 450px)") ? "bottom" : "right";

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
    [theme.breakpoints.up(450)]: {
      width: 300
    },
    [theme.breakpoints.down(450)]: {
      width: "100%"
    }
  },
  legendLabel: {
    [theme.breakpoints.up(450)]: {
      fontSize: theme.typography.h6.fontSize,
      padding: theme.spacing(1),
    },
    [theme.breakpoints.down(450)]: {
      fontSize: theme.typography.h6.fontSize,
      padding: theme.spacing(1),
      margin: "auto",
    },
  },
  trajGroup: {
    [theme.breakpoints.up(450)]: {
      flexDirection: 'row',
      paddingLeft: theme.spacing(2)
    },
    [theme.breakpoints.down(450)]: {
      flexDirection: 'row',
      paddingLeft: theme.spacing(12),
    },
  },
  flexBasis: {
    flexBasis: '40%',
  },
}));
