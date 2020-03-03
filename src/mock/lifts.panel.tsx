import React from 'react';
import ReactDOM from 'react-dom';
import LiftsPanel from '../lifts-panel';
import buildingMap from './data/building-map';
import liftStates from './data/lift-states';

ReactDOM.render(
  <LiftsPanel
    buildingMap={buildingMap}
    liftStates={liftStates}
    onLiftRequest={(lift, destination) => {
      console.log(`Request ${lift.name} to ${destination} clicked`);
    }}
  />,
  document.getElementById('root'),
);
