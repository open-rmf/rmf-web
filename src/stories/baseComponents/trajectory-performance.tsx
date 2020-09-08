import React, { useState } from 'react';
// import Trajectory from './trajectory';
import { createTrajectories } from '../baseComponents/utils-trajectory';
import { TextField } from '@material-ui/core';
import MenuItem from '@material-ui/core/MenuItem';
import Select from '@material-ui/core/Select';
import InputLabel from '@material-ui/core/InputLabel';

export default function TrajectoryPerformance() {
  const [numberOfTraj, setNumberOfTraj] = useState(100);
  const [isConflict, setIsConflict] = useState(true);
  const [trajectories, setTrajectories] = useState(createTrajectories(isConflict, numberOfTraj));

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
  );
}
