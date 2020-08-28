import React, { useState, useEffect, useCallback } from 'react';
import { AttributionControl, ImageOverlay, LayersControl, Map as LMap, Pane } from 'react-leaflet';
import * as L from 'leaflet';
import { makeStyles } from '@material-ui/core';
import { Divider, Typography, Button, Grid } from '@material-ui/core';

import RobotTrajectory, {
  RobotTrajectoryProps,
} from '../../components/schedule-visualizer/robot-trajectory';
import { viewBoxCoords, mapBound, maxBound, componentDisplayStyle } from './utils';
import {
  SettingsContext,
  TrajectoryColor,
  TrajectoryDiameter,
  defaultSettings,
  Settings,
  AnimationSpeed,
  TrajectoryAnimation,
} from '../../settings';
import RobotTrajectoriesOverlay, {
  RobotTrajectoryContext,
  RobotTrajectoriesOverlayProps,
} from '../../components/schedule-visualizer/robot-trajectories-overlay';
import {
  withFillAnimation,
  withFollowAnimation,
  withOutlineAnimation,
} from '../../components/schedule-visualizer/trajectory-animations';

const useStyles = makeStyles(() => ({
  map: {
    height: '100%',
    width: '100%',
    margin: 0,
    padding: 0,
  },
}));

export default function Trajectory(props: RobotTrajectoriesOverlayProps) {
  const classes = useStyles();
  const { conflicts, colorManager, bounds, conflictRobotNames, trajs } = props;
  const settings = React.useContext(SettingsContext);
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

  const TrajectoryComponent = React.useMemo(() => {
    const animationScale = 6000 / animDuration;
    switch (settings.trajectoryAnimation) {
      case TrajectoryAnimation.None:
        return RobotTrajectory;
      case TrajectoryAnimation.Fill:
        return withFillAnimation(RobotTrajectory, animationScale);
      case TrajectoryAnimation.Follow:
        return withFollowAnimation(RobotTrajectory, animationScale);
      case TrajectoryAnimation.Outline:
        return withOutlineAnimation(RobotTrajectory, animationScale);
    }
  }, [settings.trajectoryAnimation, animDuration]);

  return (
    <div style={componentDisplayStyle.display}>
      <div>
        <Button
          onClick={handleShowPath}
          disabled={disableButton}
          variant="contained"
          color="primary"
        >
          Draw Path
        </Button>
      </div>
      {drawAnimation ? (
        <LMap
          id="ScheduleVisualizer" // # data-* attrs are not set on the leaflet container
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
          <RobotTrajectoryContext.Provider value={{ Component: TrajectoryComponent }}>
            <RobotTrajectoriesOverlay
              bounds={bounds}
              conflicts={conflicts}
              colorManager={colorManager}
              trajs={trajs}
              conflictRobotNames={conflictRobotNames}
            />
          </RobotTrajectoryContext.Provider>
        </LMap>
      ) : (
        <div>Click on the button to draw path</div>
      )}
    </div>
  );
}
