import React from 'react';
import ReactDOM from 'react-dom';
import DoorsPanel from '../doors-panel';
import buildingMap from './data/building-map';
import doorStates from './data/door-states';

const doors = buildingMap.levels.flatMap(level => level.doors);

ReactDOM.render(
  <DoorsPanel
    doors={doors}
    doorStates={doorStates}
    onOpenClick={door => {
      console.log(`${door.name} open clicked`);
    }}
    onCloseClick={door => {
      console.log(`${door.name} close clicked`);
    }}
  />,
  document.getElementById('root'),
);
