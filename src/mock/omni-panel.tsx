import React from 'react';
import ReactDOM from 'react-dom';
import FleetManager from '../fleet-manager';
import OmniPanel, { OmniPanelView } from '../components/omni-panel';
import buildingMap from './data/building-map';
import doorStates from './data/door-states';
import liftStates from './data/lift-states';
import { FakeTransport } from './fake-transport';

const fleetManager = new FleetManager();
fleetManager.startSubscription(new FakeTransport());

ReactDOM.render(
  <OmniPanel
    initialView={OmniPanelView.MainMenu}
    buildingMap={buildingMap}
    doorStates={doorStates}
    liftStates={liftStates}
    fleetManager={fleetManager}
  />,
  document.getElementById('root'),
);
