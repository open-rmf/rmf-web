import React, { useState } from 'react';
import { Map as LMap } from 'react-leaflet';
import * as L from 'leaflet';
import { makeStyles } from '@material-ui/core';
import { Button, Typography } from '@material-ui/core';
import 'leaflet/dist/leaflet.css';

import RobotTrajectory from '../../components/schedule-visualizer/robot-trajectory';
import { mapBound, maxBound, componentDisplayStyle, StyleTyping } from './utils';
import { TrajectoryAnimation, Settings } from '../../settings';
import RobotTrajectoriesOverlay, {
  RobotTrajectoryContext,
  RobotTrajectoriesOverlayProps,
} from '../../components/schedule-visualizer/robot-trajectories-overlay';
import {
  withFillAnimation,
  withFollowAnimation,
  withOutlineAnimation,
} from '../../components/schedule-visualizer/trajectory-animations';
import { SettingsContext } from '../../components/app-contexts';

export interface TrajectoryStoryProps extends RobotTrajectoriesOverlayProps {
  description: string;
  currSettings: Settings;
}

const useStyles = makeStyles(() => ({
  map: {
    height: '100%',
    width: '100%',
    margin: 0,
    padding: 0,
  },
}));

const styles: StyleTyping = {
  placeHolderText: {
    marginTop: '2rem',
  },
  button: {
    margin: '1rem auto',
  },
};

export default function Trajectory(props: TrajectoryStoryProps) {
  const classes = useStyles();
  const {
    conflicts,
    colorManager,
    bounds,
    conflictRobotNames,
    trajs,
    description,
    currSettings,
  } = props;
  const settings = React.useContext(React.createContext(currSettings));
  const mapRef = React.useRef<LMap>(null);

  const [drawAnimation, setDrawAnimation] = useState(false);
  const [disableButton, setDisableButton] = useState(false);

  const handleShowPath = () => {
    setDrawAnimation(true);
    setDisableButton(true);

    setTimeout(() => {
      setDrawAnimation(false);
      setDisableButton(false);
    }, 5000);
  };
  const animDuration = 1000;

  const TrajectoryComponent = React.useMemo<RobotTrajectoryContext>(() => {
    const animationScale = 6000 / animDuration;
    switch (settings.trajectoryAnimation) {
      case TrajectoryAnimation.None:
        return { Component: RobotTrajectory };
      case TrajectoryAnimation.Fill:
        return { Component: withFillAnimation(RobotTrajectory, animationScale) };
      case TrajectoryAnimation.Follow:
        return { Component: withFollowAnimation(RobotTrajectory, animationScale) };
      case TrajectoryAnimation.Outline:
        return { Component: withOutlineAnimation(RobotTrajectory, animationScale) };
    }
  }, [settings.trajectoryAnimation, animDuration]);

  return (
    <div style={componentDisplayStyle.display}>
      <div style={componentDisplayStyle.modeInfoPanel}>
        <Typography variant="h6">{description}</Typography>
        <Button
          onClick={handleShowPath}
          disabled={disableButton}
          variant="contained"
          color="primary"
          style={styles.button}
        >
          Draw Trajectory
        </Button>
      </div>
      {drawAnimation ? (
        <SettingsContext.Provider value={currSettings}>
          <LMap
            id="ScheduleVisualizer"
            ref={mapRef}
            bounds={mapBound}
            maxBounds={maxBound}
            className={classes.map}
            attributionControl={false}
            crs={L.CRS.Simple}
            minZoom={4}
            maxZoom={8}
            zoomDelta={0.5}
            zoomSnap={0.5}
          >
            <RobotTrajectoryContext.Provider value={TrajectoryComponent}>
              <RobotTrajectoriesOverlay
                bounds={bounds}
                conflicts={conflicts}
                colorManager={colorManager}
                trajs={trajs}
                conflictRobotNames={conflictRobotNames}
              />
            </RobotTrajectoryContext.Provider>
          </LMap>
        </SettingsContext.Provider>
      ) : (
        <div style={styles.placeHolderText}>
          <Typography align="center" variant="h6">
            Click on the button to view trajectory
          </Typography>
        </div>
      )}
    </div>
  );
}
