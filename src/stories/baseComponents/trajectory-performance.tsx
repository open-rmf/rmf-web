import React, { useState } from 'react';
import 'leaflet/dist/leaflet.css';

import Trajectory from './trajectory';
import { createTrajectories } from '../baseComponents/utils-trajectory';
import { TextField, makeStyles } from '@material-ui/core';
import MenuItem from '@material-ui/core/MenuItem';
import Select from '@material-ui/core/Select';
import InputLabel from '@material-ui/core/InputLabel';
import { mapBound } from '../baseComponents/utils';
import ColorManager from '../../components/schedule-visualizer/colors';
import { defaultSettings } from '../../settings';

const useStyles = makeStyles(() => ({
  textField: {
    marginBottom: '1rem',
  },
}));

export default function TrajectoryPerformance() {
  const classes = useStyles();

  const [numberOfTraj, setNumberOfTraj] = useState(100);
  const [isConflict, setIsConflict] = useState(true);
  const [trajectories, setTrajectories] = useState(createTrajectories(isConflict, numberOfTraj));
  const colorManager = new ColorManager();
  const description =
    'You can test the performance of the trajectory animations by adjusting the number of trajectories and if they are conflicting in the input fields above.';

  const handleNumberOfTrajChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setNumberOfTraj(Number(e.target.value));
    setTrajectories(createTrajectories(isConflict, Number(e.target.value)));
  };

  const handleIsConflictChange = (e: React.ChangeEvent<{ value: unknown }>) => {
    e.target.value === 'true' ? setIsConflict(true) : setIsConflict(false);
    setTrajectories(createTrajectories(e.target.value === 'true' ? true : false, numberOfTraj));
  };

  return (
    <div>
      <div style={{ margin: '0 2rem' }}>
        <TextField
          placeholder="Enter number of trajectories"
          onChange={handleNumberOfTrajChange}
          defaultValue={numberOfTraj}
          className={classes.textField}
        />
        <InputLabel id="conflict">Should the trajectory conflict</InputLabel>
        <Select value={isConflict} labelId="conflict" onChange={handleIsConflictChange}>
          <MenuItem value={'true'}>Yes</MenuItem>
          <MenuItem value={'false'}>No</MenuItem>
        </Select>
      </div>
      <div>
        <Trajectory
          bounds={mapBound}
          colorManager={colorManager}
          conflicts={trajectories.conflicts}
          conflictRobotNames={trajectories.conflictingRobotName}
          overridePathColor={undefined}
          trajs={trajectories.trajectories}
          description={description}
          currSettings={defaultSettings()}
        />
      </div>
    </div>
  );
}
