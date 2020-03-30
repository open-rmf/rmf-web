import {
  Drawer,
  DrawerProps,
  FormControl,
  FormControlLabel,
  FormLabel,
  Radio,
  RadioGroup,
} from '@material-ui/core';
import React from 'react';
import { Settings, TrajectoryAnimation } from '../settings';

export interface SettingsDrawerProps extends DrawerProps {
  settings: Readonly<Settings>;
  onSettingsChange(settings: Settings): void;
}

export default function SettingsDrawer(props: SettingsDrawerProps): React.ReactElement {
  const { settings, onSettingsChange, ...otherProps } = props;
  const { trajectoryAnimation } = settings;

  const trajAnimsText = React.useMemo(
    () => Object.keys(TrajectoryAnimation).slice(Object.keys(TrajectoryAnimation).length * 0.5),
    [],
  );

  function handleTrajectoryAnimationChange(ev: React.ChangeEvent<HTMLInputElement>): void {
    const newSettings: Settings = { ...settings, trajectoryAnimation: Number(ev.target.value) };
    onSettingsChange && onSettingsChange(newSettings);
  }

  return (
    <Drawer anchor="right" {...otherProps}>
      <FormControl component="fieldset">
        <FormLabel component="legend">Trajectory Animation</FormLabel>
        <RadioGroup value={trajectoryAnimation} onChange={handleTrajectoryAnimationChange}>
          {trajAnimsText.map((text, i) => (
            <FormControlLabel key={i} value={i} control={<Radio />} label={text} />
          ))}
        </RadioGroup>
      </FormControl>
    </Drawer>
  );
}
