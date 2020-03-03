import React from 'react';
import ReactDOM from 'react-dom';
import OmniPanel, { OmniPanelView } from '../omni-panel';
import buildingMap from './data/building-map';
import doorStates from './data/door-states';
import fleets from './data/fleets';
import liftStates from './data/lift-states';

ReactDOM.render(
  <OmniPanel
    initialView={OmniPanelView.MainMenu}
    buildingMap={buildingMap}
    doorStates={doorStates}
    liftStates={liftStates}
    fleets={fleets}
  />,
  document.getElementById('root'),
);
