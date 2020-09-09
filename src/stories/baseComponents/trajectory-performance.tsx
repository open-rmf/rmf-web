import React, { useState } from 'react';
import Trajectory from './trajectory';
import { createTrajectories } from '../baseComponents/utils-trajectory';
import { TextField } from '@material-ui/core';
import MenuItem from '@material-ui/core/MenuItem';
import Select from '@material-ui/core/Select';
import InputLabel from '@material-ui/core/InputLabel';
import { mapBound } from '../baseComponents/utils';
import ColorManager from '../../components/schedule-visualizer/colors';
import { defaultSettings } from '../../settings';
import { defaultCipherList } from 'constants';

export default function TrajectoryPerformance() {
  const [numberOfTraj, setNumberOfTraj] = useState(100);
  const [isConflict, setIsConflict] = useState(true);
  const [trajectories, setTrajectories] = useState(createTrajectories(isConflict, numberOfTraj));
  const colorManager = new ColorManager();

  const handleNumberOfTrajChange = (e: any) => {
    setNumberOfTraj(e.target.value);
    setTrajectories(createTrajectories(isConflict, e.target.value));
  };

  const handleIsConflictChange = (e: any) => {
    e.target.value === 'true' ? setIsConflict(true) : setIsConflict(false);
    setTrajectories(createTrajectories(e.target.value, numberOfTraj));
  };

  return (
    <div>
      <div>
        <TextField
          placeholder="Enter number of trajectories"
          onChange={handleNumberOfTrajChange}
          defaultValue={numberOfTraj}
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
          trajs={trajectories.trajectories}
          description={''}
          currSettings={defaultSettings()}
        />
      </div>
    </div>
  );
}
