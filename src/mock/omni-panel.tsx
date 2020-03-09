import React from 'react';
import ReactDOM from 'react-dom';
import OmniPanel, { OmniPanelView } from '../components/omni-panel';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import LiftStateManager from '../lift-state-manager';
import buildingMap from './data/building-map';
import FakeTransport from './fake-transport';

const transport = new FakeTransport();
const doorStateManager = new DoorStateManager();
const liftStateManager = new LiftStateManager();
const fleetManager = new FleetManager();

ReactDOM.render(
  <OmniPanel
    transport={transport}
    initialView={OmniPanelView.MainMenu}
    buildingMap={buildingMap}
    doorStateManager={doorStateManager}
    liftStateManager={liftStateManager}
    fleetManager={fleetManager}
  />,
  document.getElementById('root'),
);
